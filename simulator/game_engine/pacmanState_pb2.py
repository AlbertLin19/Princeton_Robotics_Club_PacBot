# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: pacmanState.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x11pacmanState.proto\x12\ngameEngine\"\xd1\x06\n\x0bPacmanState\x12\x32\n\x06pacman\x18\x01 \x02(\x0b\x32\".gameEngine.PacmanState.AgentState\x12\x35\n\tred_ghost\x18\x02 \x02(\x0b\x32\".gameEngine.PacmanState.AgentState\x12\x36\n\npink_ghost\x18\x03 \x02(\x0b\x32\".gameEngine.PacmanState.AgentState\x12\x38\n\x0corange_ghost\x18\x04 \x02(\x0b\x32\".gameEngine.PacmanState.AgentState\x12\x36\n\nblue_ghost\x18\x05 \x02(\x0b\x32\".gameEngine.PacmanState.AgentState\x12.\n\x04mode\x18\x06 \x02(\x0e\x32 .gameEngine.PacmanState.GameMode\x12\x18\n\x10\x66rightened_timer\x18\x07 \x02(\x05\x12\r\n\x05score\x18\x08 \x02(\x05\x12\x31\n\x04grid\x18\t \x03(\x0e\x32#.gameEngine.PacmanState.GridElement\x12\x14\n\x0cgrid_columns\x18\n \x02(\x05\x12\r\n\x05lives\x18\x0b \x02(\x05\x12\x14\n\x0cupdate_ticks\x18\x0c \x02(\x05\x12\x18\n\x10ticks_per_update\x18\r \x02(\x05\x12\x14\n\x0c\x65lapsed_time\x18\x0e \x01(\x02\x1at\n\nAgentState\x12\t\n\x01x\x18\x01 \x02(\x05\x12\t\n\x01y\x18\x02 \x02(\x05\x12\x34\n\tdirection\x18\x03 \x01(\x0e\x32!.gameEngine.PacmanState.Direction\x12\x1a\n\x12\x66rightened_counter\x18\x04 \x01(\x05\">\n\x08GameMode\x12\t\n\x05\x43HASE\x10\x00\x12\x0b\n\x07SCATTER\x10\x01\x12\x0e\n\nFRIGHTENED\x10\x02\x12\n\n\x06PAUSED\x10\x03\"L\n\x0bGridElement\x12\x08\n\x04WALL\x10\x00\x12\n\n\x06PELLET\x10\x01\x12\x10\n\x0cPOWER_PELLET\x10\x02\x12\t\n\x05\x45MPTY\x10\x03\x12\n\n\x06\x43HERRY\x10\x04\"2\n\tDirection\x12\x06\n\x02UP\x10\x00\x12\x08\n\x04\x44OWN\x10\x01\x12\x08\n\x04LEFT\x10\x02\x12\t\n\x05RIGHT\x10\x03')



_PACMANSTATE = DESCRIPTOR.message_types_by_name['PacmanState']
_PACMANSTATE_AGENTSTATE = _PACMANSTATE.nested_types_by_name['AgentState']
_PACMANSTATE_GAMEMODE = _PACMANSTATE.enum_types_by_name['GameMode']
_PACMANSTATE_GRIDELEMENT = _PACMANSTATE.enum_types_by_name['GridElement']
_PACMANSTATE_DIRECTION = _PACMANSTATE.enum_types_by_name['Direction']
PacmanState = _reflection.GeneratedProtocolMessageType('PacmanState', (_message.Message,), {

  'AgentState' : _reflection.GeneratedProtocolMessageType('AgentState', (_message.Message,), {
    'DESCRIPTOR' : _PACMANSTATE_AGENTSTATE,
    '__module__' : 'pacmanState_pb2'
    # @@protoc_insertion_point(class_scope:gameEngine.PacmanState.AgentState)
    })
  ,
  'DESCRIPTOR' : _PACMANSTATE,
  '__module__' : 'pacmanState_pb2'
  # @@protoc_insertion_point(class_scope:gameEngine.PacmanState)
  })
_sym_db.RegisterMessage(PacmanState)
_sym_db.RegisterMessage(PacmanState.AgentState)

if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _PACMANSTATE._serialized_start=34
  _PACMANSTATE._serialized_end=883
  _PACMANSTATE_AGENTSTATE._serialized_start=573
  _PACMANSTATE_AGENTSTATE._serialized_end=689
  _PACMANSTATE_GAMEMODE._serialized_start=691
  _PACMANSTATE_GAMEMODE._serialized_end=753
  _PACMANSTATE_GRIDELEMENT._serialized_start=755
  _PACMANSTATE_GRIDELEMENT._serialized_end=831
  _PACMANSTATE_DIRECTION._serialized_start=833
  _PACMANSTATE_DIRECTION._serialized_end=883
# @@protoc_insertion_point(module_scope)