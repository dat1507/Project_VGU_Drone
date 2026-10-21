"""Phase 1 movement smoke test, before planner integration."""
import argparse
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pso_flight_demo.gazebo_runner import GazeboSession
from pso_flight_demo.screenshot import capture
p=argparse.ArgumentParser();p.add_argument('--gui',action='store_true');args=p.parse_args()
with GazeboSession('worlds/empty.sdf','runs/phase1_gui' if args.gui else 'runs/phase1_headless',gui=args.gui) as sim:
 print(json.dumps(sim.replay([[0,0,2],[2,0,2],[2,2,3]],speed=.8,fps=8,hold=4,screenshot=capture if args.gui else None),indent=2))
