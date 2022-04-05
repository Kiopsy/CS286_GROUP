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
            dir = 3
        elif vertical < 0:
            dir = 4
        else:
            dir = 0 
        ################################# End your code ###############################
        return dir

    def get_base_policy_control_component(self, taxi_state_object, ell):
        control_component = None
        # If a request's pickup location is the same as agent ell's location and the taxi is availble, return the 5+index of the request in the outstanding_requests list
        # Else if the taxi is available, return the direction of motion using the taxi ell's location and the nearest request's pickup location
        ################################# Begin your code ###############################

        # ell is the index of the agent to be used in the list: agent_locations
        ell_location = taxi_state_object.agent_locations[ell]

        manhattan_distance_lst = np.array([])

        for count, req in enumerate(taxi_state_object.outstanding_requests):
            pickup_location = [req[0], req[1]]
            if ell_location == pickup_location: 
                control_component = count + 5
                return control_component
            else:
                dist = taxi_state_object.g.manhattan_distance(ell_location, pickup_location)
                np.append(manhattan_distance_lst, dist)

        if np.any(manhattan_distance_lst):
            min = np.argmin(manhattan_distance_lst)
            nearest_req = taxi_state_object.outstanding_requests[min]

            control_component = self.next_direction(ell_location, [nearest_req[0], nearest_req[1]])

        ################################# End your code ###############################
        if control_component is None:
            return 0
        return control_component

