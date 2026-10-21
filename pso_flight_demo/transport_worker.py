"""Persistent request-only Gazebo client; isolate blocking binding from callbacks."""
import base64
import importlib
import json
import sys
from gz.transport13 import Node
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.scene_pb2 import Scene
from google.protobuf.text_format import MessageToString


def main():
    node=Node()
    for line in sys.stdin:
        try:
            item=json.loads(line)
            cls=getattr(importlib.import_module(item['module']),item['class'])
            request=cls();request.ParseFromString(base64.b64decode(item['request']))
            reply={'Boolean':Boolean,'Scene':Scene}[item['reply']]
            okay,response=node.request(item['service'],request,cls,reply,item['timeout'])
            print(json.dumps(dict(ok=okay,body=MessageToString(response,as_one_line=True))),flush=True)
        except Exception as exc:print(json.dumps(dict(ok=False,error=str(exc))),flush=True)


if __name__=='__main__':main()
