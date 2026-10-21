"""General static-world adapter using installed libsdformat frame semantics.

Research warehouse geometry.py remains unchanged for legacy callers. Primitive
collisions are conservatively enclosed in world AABBs, then sphere-inflated.
Unknown collision geometry and moving obstacles fail closed.
"""
from dataclasses import dataclass, field
from pathlib import Path
import hashlib
from itertools import product
import os
import struct
import xml.etree.ElementTree as ET
import numpy as np
from .geometry import AABB, FlightEnvironment, vector


@dataclass
class SDFWorld:
    name: str
    source: str
    sha256: str
    boxes: tuple
    floor_z: float | None
    diagnostics: list = field(default_factory=list)
    resources: dict = field(default_factory=dict)
    vehicle_names: list = field(default_factory=list)

    def environment(self, start, goal, radius=.6, margin=.25, bounds=None):
        points = np.array([vector(start),vector(goal)])
        pad=radius+margin
        if self.floor_z is not None and np.min(points[:,2]) <= self.floor_z+pad:
            raise ValueError('A/B model origins must clear floor + radius + margin')
        if bounds is None:
            low=points.min(0)-5.;high=points.max(0)+5.
            if self.floor_z is not None:low[2]=max(low[2],self.floor_z+pad+.001)
            walls={b.name.split('::')[0]:b for b in self.boxes if b.name.startswith('wall_')}
            if all(n in walls for n in ('wall_west','wall_south','wall_east','wall_north')):
                low[:2]=[walls['wall_west'].upper[0]+pad,walls['wall_south'].upper[1]+pad]
                high[:2]=[walls['wall_east'].lower[0]-pad,walls['wall_north'].lower[1]-pad]
        else:
            low,high=np.asarray(bounds,float).reshape(2,3)
            if self.floor_z is not None and low[2] <= self.floor_z+pad:
                raise ValueError('Bounds must clear inflated floor')
        return FlightEnvironment(self.boxes,low,high,radius,margin)


def xyz(v):return np.array([v.x(),v.y(),v.z()])


def mesh_vertices(path):
    if path.suffix.lower()=='.obj':
        return np.array([[float(v) for v in line.split()[1:4]] for line in path.read_text().splitlines() if line.startswith('v ')])
    if path.suffix.lower()=='.stl':
        raw=path.read_bytes()
        if len(raw)>=84 and 84+50*struct.unpack_from('<I',raw,80)[0]==len(raw):
            return np.array([struct.unpack_from('<3f',raw,i+j) for i in range(84,len(raw),50) for j in (12,24,36)])
        return np.array([[float(v) for v in line.split()[1:4]] for line in raw.decode().splitlines() if line.strip().startswith('vertex ')])
    raise ValueError(f'Unsupported mesh {path}; convert collision mesh to OBJ/STL or primitives')


def load_sdf(path, resource_paths=()):
    import sdformat14 as sdf
    from gz.math7 import Pose3d, Vector3d
    path=Path(path).resolve()
    raw=path.read_bytes()
    xml=ET.fromstring(raw)
    if xml.tag!='sdf' or len(xml.findall('world'))!=1:raise ValueError('Expected SDF containing exactly one world')
    wx=xml.find('world')
    orientation=wx.findtext('spherical_coordinates/world_frame_orientation','ENU')
    heading=float(wx.findtext('spherical_coordinates/heading_deg','0'))
    if orientation!='ENU' or heading!=0:raise ValueError('Only zero-heading ENU worlds supported')
    search=[path.parent,*map(Path,resource_paths),*map(Path,filter(None,os.environ.get('GZ_SIM_RESOURCE_PATH','').split(os.pathsep))),
            Path(os.environ.get('PSO_PX4_ROOT',str(Path.home()/'PX4-Autopilot')))/'Tools/simulation/gz/models']
    resources={}
    def find(uri):
        suffix=uri.removeprefix('model://').removeprefix('file://')
        candidates=[Path(suffix)] if Path(suffix).is_absolute() else [p/suffix for p in search]
        for candidate in candidates:
            if candidate.is_dir():candidate=candidate/'model.sdf'
            if candidate.is_file():
                resources[str(candidate.resolve())]=hashlib.sha256(candidate.read_bytes()).hexdigest()
                return str(candidate.resolve())
        return ''
    cfg=sdf.ParserConfig();cfg.set_find_callback(find)
    root=sdf.Root();root.load(str(path),cfg)
    world=root.world_by_index(0)
    if world.actor_count():raise ValueError('Animated actors are unsupported planning obstacles')
    gravity=xyz(world.gravity())
    if not np.isfinite(gravity).all() or abs(gravity[0])+abs(gravity[1])>1e-7 or gravity[2]>=0:
        raise ValueError('World requires gravity aligned with ENU -Z')
    boxes=[];floors=[];notes=[];vehicles=[]
    def walk(m,parent=Pose3d(),prefix='',inherited=False):
        name=prefix+m.name();pose=parent*m.semantic_pose().resolve()
        if m.name() in ('x500','x500_0') and not m.static():
            vehicles.append(name);notes.append(f'Excluded execution vehicle: {name}');return
        static=inherited or m.static()
        if not static:raise ValueError(f'Unsupported moving obstacle: {name}')
        for i in range(m.link_count()):
            link=m.link_by_index(i);lp=pose*link.semantic_pose().resolve()
            for j in range(link.collision_count()):
                c=link.collision_by_index(j);cp=lp*c.semantic_pose().resolve();g=c.geometry()
                rotation=np.column_stack([xyz(cp.rot().rotate_vector(Vector3d(*v))) for v in np.eye(3)])
                position=xyz(cp.pos());label=name+'::'+link.name()+'::'+c.name()
                if g.box_shape() is not None:
                    size=xyz(g.box_shape().size());vertices=np.array(list(product(*[[-s/2,s/2] for s in size])))
                elif g.sphere_shape() is not None:
                    size=np.repeat(2*g.sphere_shape().radius(),3);vertices=np.array(list(product(*[[-s/2,s/2] for s in size])))
                    notes.append(f'{label}: sphere conservatively enclosed in AABB')
                elif g.cylinder_shape() is not None:
                    shape=g.cylinder_shape();size=np.array([2*shape.radius(),2*shape.radius(),shape.length()])
                    vertices=np.array(list(product(*[[-s/2,s/2] for s in size])))
                    notes.append(f'{label}: cylinder conservatively enclosed in AABB')
                elif g.plane_shape() is not None:
                    normal=rotation@xyz(g.plane_shape().normal())
                    if not np.allclose(normal,[0,0,1]):raise ValueError(f'Unsupported non-floor plane: {label}')
                    floors.append(float(position[2]));continue
                elif g.mesh_shape() is not None:
                    mesh=g.mesh_shape();resolved=find(mesh.uri())
                    if not resolved:raise ValueError(f'Unresolved collision mesh: {mesh.uri()}')
                    if mesh.submesh():raise ValueError('Mesh submesh selection is unsupported')
                    vertices=mesh_vertices(Path(resolved))*xyz(mesh.scale())
                    size=np.ptp(vertices,axis=0);notes.append(f'{label}: mesh enclosed in transformed AABB')
                else:raise ValueError(f'Unsupported collision shape {g.type()} in {label}')
                if np.any(size<=0) or not np.isfinite(vertices).all():raise ValueError(f'Invalid collision dimensions: {label}')
                coords=vertices@rotation.T+position
                boxes.append(AABB(label,tuple(coords.min(0)),tuple(coords.max(0))))
        for i in range(m.model_count()):walk(m.model_by_index(i),pose,name+'::',static)
    for i in range(world.model_count()):walk(world.model_by_index(i))
    notes.append('All primitive and mesh obstacles use conservative world AABBs; touching counts as collision')
    return SDFWorld(world.name(),str(path),hashlib.sha256(raw).hexdigest(),tuple(boxes),max(floors) if floors else None,notes,resources,vehicles)
