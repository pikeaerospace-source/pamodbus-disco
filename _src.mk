# /********************************************************************************
# * ╔═╗┬┬┌─┌─┐  ╔═╗┌─┐┬─┐┌─┐┌─┐┌─┐┌─┐┌─┐┌─┐                                       *
# * ╠═╝│├┴┐├┤   ╠═╣├┤ ├┬┘│ │└─┐├─┘├─┤│  ├┤                                        *
# * ╩  ┴┴ ┴└─┘  ╩ ╩└─┘┴└─└─┘└─┘┴  ┴ ┴└─┘└─┘                                       *
# * ╦═╗┌─┐┌─┐┌─┐┌─┐┬─┐┌─┐┬ ┬  ╔═╗┌─┐                                              *
# * ╠╦╝├┤ └─┐├┤ ├─┤├┬┘│  ├─┤  ║  │ │                                              *
# * ╩╚═└─┘└─┘└─┘┴ ┴┴└─└─┘┴ ┴  ╚═╝└─┘o                                             *
# *                                                                               *
# * Copyright © 2025 Pike Aerospace Research Co.                                  *
# *                                                                               *
# ********************************************************************************/
# pamodbus-disco - Pike Aero Modbus Discovery Library
#
# Pure-protocol MODBUS discovery library built on pamodbus.
# Provides master and slave discovery state machines for automatic
# MODBUS slave ID assignment on a bus.
#
# Dependencies: pamodbus

SRC_PAMODBUS_DISCO=${SRC_ROOT}/pamodbus-disco
INC += -I $(SRC_PAMODBUS_DISCO)/include
INC += -I $(SRC_PAMODBUS_DISCO)/src

SRCS_CC += $(SRC_PAMODBUS_DISCO)/src/pa_disco_master.c
SRCS_CC += $(SRC_PAMODBUS_DISCO)/src/pa_disco_slave.c
SRCS_CC += $(SRC_PAMODBUS_DISCO)/src/pa_disco_list.c
SRCS_CC += $(SRC_PAMODBUS_DISCO)/src/pa_disco_register_map.c

# modbus-disco compatibility layer
SRCS_CC += $(SRC_PAMODBUS_DISCO)/src/modbus-disco-list-compat.c
