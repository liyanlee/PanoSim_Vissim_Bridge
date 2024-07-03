import os
import math
from ctypes import *
from time import sleep

from TrafficModelInterface import *
from BusAccessor import *


class EgoData2Vissim(Structure):
    _fields_ = [
        ('ID', c_int),
        ('Type', c_int),
        ('X', c_double),
        ('Y', c_double),
        ('Z', c_double),
        ('Yaw', c_double),
        ('Pitch', c_double),
        ('Speed', c_double),
        ('Create', c_bool),
        ('CreateID', c_int),
        ('Delete', c_bool),
        ('ControlledByVissim', c_bool),
        ('RoutingDecisionNo', c_long),
        ('RouteNo', c_long)
    ]


class VissimTrafficData(Structure):
    _fields_ = [
        ('ID', c_long),
        ('Type', c_long),
        ('ModelFileName', c_char * 100),
        ('color', c_long),
        ('X', c_double),
        ('Y', c_double),
        ('Z', c_double),
        ('Yaw', c_double),
        ('Pitch', c_double),
        ('Speed', c_double),
        ('LeadingVehicleID', c_long),
        ('TrailingVehicleID', c_long),
        ('LinkID', c_long),
        ('LinkName', c_char * 100),
        ('LinkCoordinate', c_double),
        ('LaneIndex', c_int),
        ('TurningIndicator', c_int),
        ('PreviousIndex', c_long),
        ('NumUDAs', c_long),
        ('UDA', c_double * 16),
        ('CreateID', c_int),
        ('ControlledByVissim', c_bool)
    ]


class VissimSignalData(Structure):
    _fields_ = [
        ('CtrlId', c_long),
        ('GroupId', c_long),
        ('State', c_int)
    ]


def ModelStart(userData):
    print(userData)
    bus_ego_format = 'time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d'
    userData['bus_ego'] = BusAccessor(userData['busId'], 'ego', bus_ego_format)
    userData['last'] = 0

    userData['vissim_ids'] = {}

    # D:/Tools/PTV_Vision/Vissim2024/API/DrivingSimulator_DLL/bin/x64/DrivingSimulatorProxy.dll
    file_dll = userData['parameters']['vissim_dll']
    userData['vissim_api'] = cdll.LoadLibrary(file_dll)
    # D:\PanoSim5\PanoSimDatabase\Plugin\Disturbance\Vissim\PanoTown.inpx
    file_inpx = userData['parameters']['inpx']
    result = userData['vissim_api'].VISSIM_Connect(
        c_ushort(2400), os.path.abspath(file_inpx),
        c_ushort(100), c_double(-1),
        c_ushort(100), c_ushort(0), c_ushort(100),
        c_ushort(100), c_ushort(0), c_ushort(100)
    )
    if not result:
        raise RuntimeError('There was an error when establishing a connection with PTV-Vissim')


def State2String(state):
    if state == 1:
        return 'Red'
    elif state == 2:
        return 'Red+Amber'
    elif state == 3:
        return 'Green'
    elif state == 4:
        return 'Amber'
    elif state == 5:
        return 'Off (black)'
    elif state == 6:
        return 'Undefined'
    elif state == 7:
        return 'Flashing Amber'
    elif state == 8:
        return 'Flashing Red'
    elif state == 9:
        return 'Flashing Green'
    elif state == 10:
        return 'Alternating Red/Green'
    elif state == 11:
        return 'Green+Amber'

    return str(state)


def State2Enum(state):
    if state == 1:
        return traffic_light_state.red
    elif state == 3:
        return traffic_light_state.green
    return traffic_light_state.yellow


def Type2Enum(type):
    if type == 300:
        return vehicle_type.Bus
    elif type == 190:
        return vehicle_type.Van
    elif type == 200:
        return vehicle_type.Van
    return vehicle_type.Car


def ModelOutput(userData):
    ts, ego_x, ego_y, ego_z, ego_yaw, ego_pitch, _, ego_speed = userData['bus_ego'].readHeader()

    ego_data = (EgoData2Vissim * 1)(*[(1, 630, ego_x, ego_y, ego_z, ego_yaw, ego_pitch, ego_speed, False, 1, False, False, 0, 0)])
    userData['vissim_api'].VISSIM_SetDriverVehicles(1, byref(ego_data))

    if ts > 10000:
        userData['last'] = ts

        vehicle_count = c_int(0)
        vehicles = POINTER(VissimTrafficData)()
        userData['vissim_api'].VISSIM_GetTrafficVehicles(byref(vehicle_count), byref(vehicles))

        signal_count = c_int(0)
        signals = POINTER(VissimSignalData)()
        userData['vissim_api'].VISSIM_GetSignalStates(byref(signal_count), byref(signals))
        for i in range(signal_count.value):
            signal = signals[i]
            if signal.GroupId == 1:
                setTrafficLightState(1, next_junction_direction.straight, State2Enum(signal.State), 1)
                setTrafficLightState(3, next_junction_direction.straight, State2Enum(signal.State), 1)
            else:
                setTrafficLightState(2, next_junction_direction.straight, State2Enum(signal.State), 1)
                setTrafficLightState(4, next_junction_direction.straight, State2Enum(signal.State), 1)

        ids = getVehicleList()
        for i in range(vehicle_count.value):
            v = vehicles[i]

            if v.ControlledByVissim:
                id = userData['vissim_ids'].get(v.ID, -1)
                if id > 0:
                    if id in ids:
                        moveTo(id, v.X, v.Y, 90 - math.degrees(v.Yaw))
                    else:
                        new_id = addVehicle(v.X, v.Y, v.Speed, Type2Enum(v.Type))
                        if new_id > 0:
                            userData['vissim_ids'][v.ID] = new_id
                else:
                    new_id = addVehicle(v.X, v.Y, v.Speed, Type2Enum(v.Type))
                    if new_id > 0:
                        userData['vissim_ids'][v.ID] = new_id


def ModelTerminate(userData):
    userData['vissim_api'].VISSIM_Disconnect()
    sleep(1)
