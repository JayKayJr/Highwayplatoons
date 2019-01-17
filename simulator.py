#!/usr/bin/env python
#
# Copyright (c) 2017 Michele Segata <segata@ccs-labs.org>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see http://www.gnu.org/licenses/.
#
import pdb
import os
import sys
import ccparams as cc
import random
from utils import add_vehicle, set_par, change_lane, communicate, \
    get_distance, get_par, start_sumo, running, validate_params, retrieve_vehicles, \
    filter_cacc_vehicles

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib

import traci

# vehicle length
LENGTH = 4
# inter-vehicle distance
DISTANCE = 7
# inter-vehicle distance when leaving space for joining
JOIN_DISTANCE = DISTANCE * 2
REJOIN_DISTANCE = DISTANCE * 1.5
# cruising speed
SPEED = 130/3.6

# maneuver states:
GOING_TO_POSITION = 0
OPENING_GAP = 1
COMPLETED = 2
SPLITTING= 3
INOUT = 4
PLATOONTERMINATED = 5


N_VEHICLES = 8
# maneuver actors
LEADER = "v.0"
JOINER = "v.%d" % N_VEHICLES



#JOIN_POSITION = 0




# sumo launch command
sumoBinary = sumolib.checkBinary('sumo-gui')
sumoCmd = [sumoBinary, "-D", "-c", "cfg/freeway.sumo.cfg"]


def add_vehicles(n, real_engine=False):
    """
    Adds a platoon of n vehicles to the simulation, plus an additional one
    farther away that wants to join the platoon
    :param n: number of vehicles of the platoon
    :param real_engine: set to true to use the realistic engine model,
    false to use a first order lag model
    :return: returns the topology of the platoon, i.e., a dictionary which
    indicates, for each vehicle, who is its leader and who is its front
    vehicle. The topology can the be used by the data exchange logic to
    automatically fetch data from leading and front vehicle to feed the CACC
    """
    # add a platoon of n vehicles
    topology = {}
    for i in range(n):
        vid = "v.%d" % i
        add_vehicle(vid, (n - i + 1) * (DISTANCE + LENGTH) + 50, 3, SPEED, DISTANCE,
                    real_engine)
        change_lane(vid, 3)
        if i == 0:
            set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
            traci.vehicle.setRouteID(vid, "platoon_route")
        else:
            set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
            traci.vehicle.setRouteID(vid, "platoon_route")
        if i > 0:
            topology[vid] = {"front": "v.%d" % (i - 1), "leader": LEADER}
    # add a vehicle that wants to join the platoon
    vid = "v.%d" % n
    joiner_placement = (LENGTH + DISTANCE)*N_VEHICLES
    add_vehicle(vid, 10 , 2, SPEED, DISTANCE, real_engine)
    change_lane(vid, 2)
    set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
    set_par(vid, cc.PAR_CACC_SPACING, JOIN_DISTANCE)
    traci.vehicle.setRouteID(vid, "loner")
    return topology

def join_position(LEADER, jid):
    """This function dictates the position the joiner enters the platoon"""
    leader_pos = traci.vehicle.getLanePosition(LEADER)
    joiner_pos = traci.vehicle.getLanePosition(jid)
    join_pos = 0
    if leader_pos > joiner_pos:
        if leader_pos >= joiner_pos + ((LENGTH + DISTANCE)*N_VEHICLES):
            join_pos = 8
            return join_pos

        elif leader_pos <= 0.6*(joiner_pos + (LENGTH + DISTANCE)*N_VEHICLES):
            join_pos = 4
            return join_pos

        else:
            join_pos = 0
            return join_pos 
 

def veh_ordering(jid, JOIN_POSITION):
    if JOIN_POSITION == 8:
        index = int(jid.split(".")[1])
        front_join = "v.%d" % int(JOIN_POSITION-1)
        behind_join = jid
        ORDERING = (front_join, behind_join)
        return ORDERING

    if JOIN_POSITION == 4:
        index = int(jid.split(".")[1])
        front_join = "v.%d" % (JOIN_POSITION - 1)
        behind_join = "v.%d" % JOIN_POSITION
        ORDERING = (front_join, behind_join)
        return ORDERING

    elif JOIN_POSITION == 0:
        
        front_join = None
        behind_join = "v.0"
        ORDERING = (front_join, behind_join)
        return ORDERING

def get_in_position(jid, fid, bid, topology, JOIN_POSITION):
    """
    Makes the joining vehicle get close to the join position. This is done by
    changing the topology and setting the leader and the front vehicle for
    the joiner. In addition, we increase the cruising speed and we switch to
    the "fake" CACC, which uses a given GPS distance instead of the radar
    distance to compute the control action
    :param jid: id of the joiner
    :param fid: id of the vehicle that will become the predecessor or successor of the joiner
    :param topology: the current platoon topology
    :return: the modified topology
    """
    """"""

    if not JOIN_POSITION == 0: # if join pos is not leader position join veh has front veh
        global LEADER
        fid = "v.%d" % int((JOIN_POSITION - 1))
        topology[jid] = {"leader": LEADER, "front": fid}
        set_par(jid, cc.PAR_CC_DESIRED_SPEED, SPEED + 15)
        set_par(jid, cc.PAR_ACTIVE_CONTROLLER, cc.FAKED_CACC)
        return topology

    else:
        LEADER = jid
        index = int(jid.split(".")[1])
        for i in range(1, index):
            vid = "v.%d" % i
            topology[vid] = {"front": "v.%d" % (i - 1), "leader": LEADER}
            set_par(jid, cc.PAR_CC_DESIRED_SPEED, SPEED + 15)
            set_par(jid, cc.PAR_ACTIVE_CONTROLLER, cc.FAKED_CACC)
            return topology 
    return topology

def rear_join_completion(jid, topology):  

    """Switches active controller of joiner from fake to cacc"""
    index = int(jid.split(".")[1])
    topology[jid] = {"front": "v.%d" % (index - 1), "leader": LEADER}
    set_par(jid, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
    set_par(jid, cc.PAR_CACC_SPACING, DISTANCE)
    change_lane(JOINER, 3)

def rear_exit(jid, cacc_vehicles, topology):
    """increases gap between joiner and front vehicle and prepares the join vehicle to exit the platoon """
    for i in range(0, len(cacc_vehicles)):
        LEADER = "v.0"
        vid = "v.%d" % i
        if i > 0:
            topology[vid] = {"front": "v.%d" % (i - 1), "leader": LEADER} 
    set_par(jid, cc.PAR_CACC_SPACING, JOIN_DISTANCE)
    set_par(jid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
    return topology
def platoonvehs(vehicles):

    platoon_vehicles = []


    for vehicle in vehicles:
        edge = traci.vehicle.getRoadID(vehicle)
        route = traci.vehicle.getRoute(vehicle)
        next_edge = route[route.index(edge) + 1]
        platoon_route = traci.vehicle.getRoute('v.0')
        next_platoon_edge = platoon_route[platoon_route.index(edge) + 1]

        if next_edge == next_platoon_edge:
            platoon_vehicles.append(vehicle)
    print(platoon_vehicles)
        
    return platoon_vehicles

def rear_exit_reset(platoon_vehicles):

    print(platoon_vehicles)
    
    LEADER = 'v.0'
    topology = {}


    for vehicle in platoon_vehicles:
        vid =vehicle
        index = int(vid.split(".")[1])
        if index > 0:
            topology[vid] = {"front": "v.%d" % (index - 1), "leader": LEADER}
            set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
            set_par(vid, cc.PAR_CACC_SPACING, DISTANCE)
        if index == 0:
            set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
    return topology




def free_joiner(jid):
    set_par(jid,cc.PAR_ACTIVE_CONTROLLER, cc.ACC)




def front_join(jid):
    """Switches active controller of joiner from fake to cacc and resets platoon topology"""
    
    change_lane(JOINER, 3)
    set_par(jid, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
    set_par(jid, cc.PAR_CACC_SPACING, JOIN_DISTANCE)

def front_join_reset(jleaderid):
    index = int(jid.split(".")[1])
    LEADER = jid
    for i in range(0,index):
        vid ="v.%d" % i
        if i > 0:
            topology[vid] = {"front": "v.%d" % (index - 1), "leader": LEADER}
            set_par(vid, cc.PAR_CACC_SPACING, DISTANCE)
        if i == 0:
            topology[vid] = {"leader": LEADER, "front": LEADER}

    return topology


def open_gap(vid, jid, topology, n):
    """
    Makes the vehicle that will be behind the joiner open a gap to let the
    joiner in. This is done by creating a temporary platoon, i.e., setting
    the leader of all vehicles behind to the one that opens the gap and then
    setting the front vehicle of the latter to be the joiner. To properly
    open the gap, the vehicle leaving space switches to the "fake" CACC,
    to consider the GPS distance to the joiner
    :param vid: vehicle that should open the gap
    :param jid: id of the joiner
    :param topology: the current platoon topology
    :param n: total number of vehicles currently in the platoon
    :return: the modified topology
    """
    index = int(vid.split(".")[1])
    for i in range(index + 1, n):
        # temporarily change the leader
        topology["v.%d" % i]["leader"] = vid
    # the front vehicle of the vehicle opening the gap is the joiner
    topology[vid]["front"] = jid
    set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.FAKED_CACC)
    set_par(vid, cc.PAR_CACC_SPACING, JOIN_DISTANCE)
    return topology

def reopen_gap(vid, jid, topology, n):
    """
    Makes the vehicle that will be behind the joiner open a gap to let the
    joiner in. This is done by creating a temporary platoon, i.e., setting
    the leader of all vehicles behind to the one that opens the gap and then
    setting the front vehicle of the latter to be the joiner. To properly
    open the gap, the vehicle leaving space switches to the "fake" CACC,
    to consider the GPS distance to the joiner
    :param vid: vehicle that should open the gap
    :param jid: id of the joiner
    :param topology: the current platoon topology
    :param n: total number of vehicles currently in the platoon
    :return: the modified topology
    """
    index = int(vid.split(".")[1])
    for i in range(index + 1, n-1):
        # temporarily change the leader
        topology["v.%d" % i]["leader"] = vid
    # the front vehicle of the vehicle opening the gap is the joiner
    topology[vid]["front"] = jid
    set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.FAKED_CACC)
    set_par(vid, cc.PAR_CACC_SPACING, JOIN_DISTANCE)
    set_par(jid, cc.PAR_CACC_SPACING, JOIN_DISTANCE)
    return topology


def close_gap(vid, topology, n):
    """resets the leader of Behind join and succeeding vehicles and closes the gap between them and the leader"""
    index = int(vid.split(".")[1])

    for i in range(index + 1, n-1):
        if i > 0:
            topology[vid] = {"front": "v.%d" % (i - 1), "leader": vid} 

    set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.FAKED_CACC)
    set_par(vid, cc.PAR_CACC_SPACING, REJOIN_DISTANCE)
    #set_par(vid, cc.PAR_CC_DESIRED_SPEED, SPEED)
    return topology

def gap_closed_center_reset(vid, topology, n):
    """Resets Behind vehicle and successors to """
    index = int(vid.split(".")[1])

    for i in range(index, n-1):
        if i > 0:
            topology[vid] = {"front": "v.%d" % (i - 1), "leader": LEADER} 

    set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
    set_par(vid, cc.PAR_CACC_SPACING, DISTANCE)
    #set_par(vid, cc.PAR_CC_DESIRED_SPEED, SPEED)
    return topology



def leader_exit_reset(cacc_vehicles):
    """Resets leadership to first vehicle after leader and resets platoon topology """

    for i in range(0, len(cacc_vehicles)):
        LEADER = cacc_vehicles[-1]
        vid = "v.%d" % i
        if i > 0:
            topology[vid] = {"front": "v.%d" % (i - 1), "leader": LEADER} 
    set_par(LEADER, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
    set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
    set_par(vid, cc.PAR_CACC_SPACING, DISTANCE)
    #set_par(vid, cc.PAR_CC_DESIRED_SPEED, SPEED)
    return topology

def reset_leader_center_join(vid, topology, n):
    """
    After the maneuver is completed, the vehicles behind the one that opened
    the gap, reset the leader to the initial one
    :param vid: id of the vehicle that let the joiner in
    :param topology: the current platoon topology
    :param n: total number of vehicles in the platoon (before the joiner)
    :return: the modified topology
    """
    index = int(vid.split(".")[1])
    for i in range(index + 1, n):
        # temporarily change the leader
        topology["v.%d" % i]["leader"] = LEADER
    return topology


def look_4_splits(vehicles):
    """
    Checks if any vehicle of the platoon wants to leave and prepares the
    platoon for the maneuver in that case
    """
    for i in range(len(vehicles)):
        edge = traci.vehicle.getRoadID(vehicles[i])
        # If vehicle is in an internal edge don't check for splits. If a
        # split is coming it should have been noted before
        if edge.startswith(':'):
            continue

        route = traci.vehicle.getRoute(vehicles[i])

        if route.index(edge) == len(route) - 1:
            continue

        next_edge = route[route.index(edge) + 1]
        platoon_route = traci.vehicle.getRoute(vehicles[0])

        if edge not in platoon_route or platoon_route.index(edge) + 1 >= len(platoon_route):
            continue

        next_platoon_edge = platoon_route[platoon_route.index(edge) + 1]

        if next_edge != next_platoon_edge:
            # TODO Don't hardcode the distance and make it speed dependant
            if traci.vehicle.getDrivingDistance(vehicles[i], next_edge, 3) < 2000.0:
                print("SPLIT FOUND!!!!!!") 
                return True


def it_is_safe_to_change_lane(vehicle, lane_id, safe_gap):
    """
    Checks whether it is safe or not for a vehicle to change to a lane
    :param vehicle: ID of the vehicle that wants to change lane
    :param lane_id: ID of the destination lane
    :param safe_gap: minimum gap to consider that the lane change is safe
    :type vehicle: str
    :type lane_id: str
    :type safe_gap: float
    :return: True if it is safe to change lane, False otherwise
    :rtype: bool
    """
    vehicles_in_lane = traci.lane.getLastStepVehicleIDs(lane_id)

    for v in vehicles_in_lane:
        gap, valid_gap = gap_between_vehicles(vehicle, v)
        if gap < safe_gap and valid_gap:
            return False

    return True

def platoon_exit_protocol(platoon_vehicles):
    #defines a function that takes out vehicles that almost arriving from the simulation
    #it takes in any vehicles that are within a distance to the end of their routes 
    #and takes them out of the listr of running vehicles 
    #param platoon_vehicles: running platoon vehicles

    topology={}

    for vehicle in platoon_vehicles:
        set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
        
    return topology




def main(demo_mode, real_engine, setter=None):
    # used to randomly color the vehicles
    random.seed(1)
    start_sumo("cfg/freeway.sumo.cfg", False)
    step = 0
    state = GOING_TO_POSITION
    edge_filter, vtype_filter = validate_params(edge_filter= ['0','1','2','3','4','5', '6', '7', '8', '9', '10','11', 'e4'] , vtype_filter= ["vtypeauto","vtypeloner"])
    while running(demo_mode, step, 6000):

        # when reaching 60 seconds, reset the simulation when in demo_mode
        if demo_mode and step == 6000:
            start_sumo("cfg/freeway.sumo.cfg", True)
            step = 0
            state = GOING_TO_POSITION
            random.seed(1)

        traci.simulationStep() 
        vehicles = retrieve_vehicles(edge_filter)
        cacc_vehicles = filter_cacc_vehicles(vehicles, vtype_filter)
        simulation_vehicles = traci.vehicle.getIDList()

        

        if step == 0:
            # create vehicles and track the joiner
            topology = add_vehicles(N_VEHICLES, real_engine)
            print(topology)
            traci.gui.trackVehicle("View #0", LEADER)
            traci.gui.setZoom("View #0", 4500)
            # Got to figure out where to call join_pos to avoid ref before assignment error
            
            
            
        if step % 10 == 1:
            # simulate vehicle communication every 100 ms
            communicate(topology)
        if step == 100:
            # at 1 second, let the joiner get closer to the platoon
            JOIN_POSITION = join_position(LEADER, JOINER)
            print(JOIN_POSITION)
            ORDER = veh_ordering(JOINER, JOIN_POSITION)
            print(ORDER)
            FRONT_JOIN = ORDER[0]
            BEHIND_JOIN = ORDER[1]
            print(FRONT_JOIN)
            print(BEHIND_JOIN)
            
            topology = get_in_position(JOINER, FRONT_JOIN, BEHIND_JOIN, topology, JOIN_POSITION)
            print(topology)
        if state == GOING_TO_POSITION and step > 100:
            # when the distance of the joiner is small enough, let the others
            # open a gap to let the joiner enter the platoon
            if JOIN_POSITION == 8:
                if get_distance(JOINER, FRONT_JOIN) < JOIN_DISTANCE + 1:
                    rear_join_completion(JOINER, topology)
                    print("REAR JOIN COMPLETED")
                    state= COMPLETED

            if JOIN_POSITION == 4:
                if get_distance(JOINER, FRONT_JOIN) < JOIN_DISTANCE + 1:
                    state = OPENING_GAP
                    topology = open_gap(BEHIND_JOIN, JOINER, topology, N_VEHICLES)
                    if get_distance(BEHIND_JOIN, FRONT_JOIN) > 2 * JOIN_DISTANCE + 2:
                        
                        
                        set_par(JOINER, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
                        set_par(JOINER, cc.PAR_CACC_SPACING, DISTANCE)
                        change_lane(JOINER, 3)#replace 0 with lane num for flex
                        set_par(BEHIND_JOIN, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
                        set_par(BEHIND_JOIN, cc.PAR_CACC_SPACING, DISTANCE)
                        topology = reset_leader_center_join(BEHIND_JOIN, topology, N_VEHICLES)
                        state = COMPLETED

            elif JOIN_POSITION == 0:
                if get_distance(JOINER, BEHIND_JOIN) == -1*JOIN_DISTANCE:
                    front_join(JOINER)
                    front_join_reset(JOINER)
                    state = COMPLETED

        if state == COMPLETED:
            if look_4_splits(vehicles):    

                ##start here code all exit maneuvers here
                #########################################

                if JOIN_POSITION == 8:
                    print(JOIN_POSITION)
                    topology = rear_exit(JOINER, cacc_vehicles, topology)
                    state=SPLITTING
                    
                    print(topology)
                if JOIN_POSITION == 4:

                    topology = reopen_gap(BEHIND_JOIN, JOINER, topology, N_VEHICLES)
                    state=SPLITTING
                    print(topology)

                else:
                    pass

        if state == SPLITTING: #and get_distance(JOINER, FRONT_JOIN) > JOIN_DISTANCE:

            if JOIN_POSITION == 4:
            
                if get_distance(JOINER, FRONT_JOIN) > JOIN_DISTANCE:
                    change_lane(JOINER, 3)
                    set_par(JOINER, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
            
                    topology = close_gap(BEHIND_JOIN, topology,N_VEHICLES)
                
                if get_distance(BEHIND_JOIN, FRONT_JOIN) == DISTANCE+1:
                    topology = gap_closed_center_reset(BEHIND_JOIN, topology, N_VEHICLES)
                    state = INOUT

            if JOIN_POSITION == 8:


                if get_distance(JOINER, FRONT_JOIN) > JOIN_DISTANCE:
                    change_lane(JOINER, 0)
                    platoon_vehicles = platoonvehs(vehicles)
                    topology = rear_exit_reset(platoon_vehicles)
                    free_joiner(JOINER)
                    state = INOUT
                    print("FREE AT LAST")

            if JOIN_POSITION == 0:
                change_lane(JOINER,3)
                topology = leader_exit_reset(cacc_vehicles)
                state == INOUT

        if state == INOUT:
            if traci.vehicle.getDrivingDistance2D(LEADER, 5.45,-1497.99) < 600.0:
                print("LEADER CHANGE LANE EMINENT!!!")
                platoon_vehicles = platoonvehs(vehicles)

                for vehicle in platoon_vehicles:
                    change_lane(vehicle, 0)
                topology = rear_exit_reset(platoon_vehicles)

                #for vehicle in platoon_vehicles:
                     #if next_edge.split[1] == 'x':

                topology = platoon_exit_protocol(platoon_vehicles)
                state = PLATOONTERMINATED




        step += 1
        print(step)
        print(state)
        print(JOINER)
        print(LEADER)
        print(topology)
        #print(platoon_vehicles)
        print(simulation_vehicles)
        print(cacc_vehicles)
        print(vehicles)

    traci.close()


if __name__ == "__main__":
    main(True, True)
