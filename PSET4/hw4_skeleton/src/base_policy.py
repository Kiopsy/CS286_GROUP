from flask import request
import numpy as np

class base_policy:
    def get_control(self, taxi_state_object):
        #print('get_control() base policy')
        control = []
        for ell in range(taxi_state_object.g.m):
            control.append(self.get_base_policy_control_component(taxi_state_object, ell))
        return control

    def next_direction(self, location1, location2):
        dir = 0
        # Return the direction of motion 0:stay, 1:left, 2:right, 3:up, 4:down based on the minimum manhattan distance between location1, location2
        # You can give horizontal motion more presidence over vertical motion
        # for example location1 = (1, 1), location2 = (2, 3), the fuction returns 2.
        ################################# Begin your code ###############################
        horizontal = location2[0] - location1[0]
        vertical = location2[1] - location1[1]

        if horizontal > 0:
            dir = 2
        elif horizontal < 0: 
            dir = 1
        elif vertical > 0:
            dir = 4
        elif vertical < 0:
            dir = 3
        else:
            dir = 0 
        ################################# End your code ###############################

        return dir

    def get_base_policy_control_component(self, taxi_state_object, ell):
        control_component = 0
        # If a request's pickup location is the same as agent ell's location and the taxi is availble, return the 5+index of the request in the outstanding_requests list
        # Else if the taxi is available, return the direction of motion using the taxi ell's location and the nearest request's pickup location
        ################################# Begin your code ###############################

        # Create variable for the index of the agent to be used in the list: agent_locations
        ell_loc = taxi_state_object.agent_locations[ell]
        # Create variable for time left in ell's ride and determine whether ell is available
        time_left = taxi_state_object.time_left_in_current_trip[ell]
        ell_avail = True if time_left == 0 else False

        if ell_avail:
            # Create list to save Manhattan distances
            manhattan_dists = []

            # Consider all outstanding requests
            for idx, req in enumerate(taxi_state_object.outstanding_requests):
                pickup_loc = [req[0], req[1]]
                # print(str(count) + "th pickup location: " + str(pickup_location))
                if ell_loc == pickup_loc:
                    # Pickup request in ell's current location
                    control_component = 5 + idx
                    return control_component
                else:
                    # Add Manhattan distance to list for future control determination
                    dist = taxi_state_object.g.manhattan_distance(ell_loc, pickup_loc)
                    # print("dist: " + str(dist))
                    manhattan_dists.append(dist)
            
            print(manhattan_dists)
            if manhattan_dists:
                # Find which request is closest (index of minimum distance)
                min_idx = manhattan_dists.index(min(manhattan_dists))
                # print("m: " + str(m))
                nearest_req = taxi_state_object.outstanding_requests[min_idx]

                control_component = self.next_direction(ell_loc, (nearest_req[0], nearest_req[1]))
                # print("s': " + str(control_component))

            
        ################################# End your code ###############################
        return control_component

