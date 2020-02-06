#!/usr/bin/python

from indydcp import indydcp_client
from indydcp import indy_program_maker

import json


_bind_ip = "192.168.3.104"
_robot_ip = "192.168.3.117"
_name = "NRMK-Indy7"


indy = indydcp_client.IndyDCPClient(_bind_ip, _robot_ip, _name)
indy.connect()


prog = indy_program_maker.JsonProgramComponent(policy=0, resume_time=2)
    
prog.add_joint_move_to([0, 0, 0, 0, 0, 10], vel=1, blend=0)

json_string = json.dumps(prog.json_program)
indy.set_and_start_json_program(json_string)


indy.disconnect()