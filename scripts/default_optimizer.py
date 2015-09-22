#!/usr/bin/env python

import rospy
import dynamic_bandwidth_manager
import pulp
import numpy as np

def optimize(managed_topics, priorities):
    # Initialize the model
    model = pulp.LpProblem("Dynamic Bandwidth Optmization for Multirobot Systems", pulp.LpMaximize)

    # Create a dictionary of pulp variables with keys from managed topics
    # the default lower bound is -inf
    f = pulp.LpVariable.dict('f_%s', managed_topics, lowBound=0)

    # cost data
    cost = dict(zip(managed_topics, (dynamic_bandwidth_manager.DBMParam.get_message_size_in_bytes(topic) for topic in managed_topics)))

    # Create the objective
    #model += sum(cost[i] * f[i] for i in managed_topics)
    for topic in managed_topics:
        model += f[topic]

    available_bandwidth_in_mbits = dynamic_bandwidth_manager.DBMParam.get_max_bandwidth_in_mbits() * dynamic_bandwidth_manager.DBMParam.get_max_bandwidth_utilization()
    available_bandwidth_in_bytes = available_bandwidth_in_mbits * 1310720

    # Constraints:
    # The total bandwidth should be lower than max bandwidth available
    model += sum(cost[i] * f[i] for i in managed_topics) <= available_bandwidth_in_bytes

    # The frequency should be higher than minimum frequency
    # Calculating the minimum frequency in relation to the priority
    fmin = dict(zip(managed_topics,
                (((dynamic_bandwidth_manager.DBMParam.get_max_frequency(topic) - dynamic_bandwidth_manager.DBMParam.get_min_frequency(topic)) * priorities[topic] + dynamic_bandwidth_manager.DBMParam.get_min_frequency(topic))
                for topic in managed_topics)))

    for topic in managed_topics:
            model += f[topic] >= fmin[topic]

    # The frequency should be lower than maximum frequency
    fmax = dict(zip(managed_topics,
        (dynamic_bandwidth_manager.DBMParam.get_max_frequency(topic) for topic in managed_topics)))
    for topic in managed_topics:
        model += f[topic] <= fmax[topic]

    # Problem is then solved with the default solver
    model.solve()

    # Create the result dictionary
    result = {}
    if model.status == 1:
        for topic in managed_topics:
            result[topic] = f[topic].value()

    print 'Available bandwidth: (%s mbits / %s bytes)' %(available_bandwidth_in_mbits, available_bandwidth_in_bytes)
    print 'fmin: %s' %(fmin)
    print 'fmax: %s' %(fmax)
    if model.status == 1: print 'Result: %s' %(result)
    else: print 'Result: %s' %(result)

    return result
        
if __name__ == '__main__':
    try:
        rospy.init_node('default_optimizer', anonymous=True)
        optimizer = dynamic_bandwidth_manager.DBMOptimizer(optimize)
        optimizer.start()

    except rospy.ROSInterruptException: pass