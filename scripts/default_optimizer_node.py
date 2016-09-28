#!/usr/bin/env python

import rospy
import dynamic_bandwidth_manager as DBM
import pulp
import numpy as np

def priorities_are_equal(priorities):
    return len(set(priorities.values())) == 1

def split_bandwidth_equally(managed_topics, available_bandwidth_in_bytes):
    result = {}
    for topic in managed_topics:
        result[topic] = available_bandwidth_in_bytes / len(managed_topics) / DBM.DBMParam.get_message_size_in_bytes(topic)
        if result[topic] > DBM.DBMParam.get_max_frequency(topic):
            result[topic] = DBM.DBMParam.get_max_frequency(topic)

    return result

def solve_optimization(managed_topics, priorities, available_bandwidth_in_bytes):
    # Initialize the model
    model = pulp.LpProblem("Dynamic Bandwidth Optmization for Multirobot Systems", pulp.LpMaximize)

    # Create a dictionary of pulp variables with keys from managed topics
    # the default lower bound is -inf
    f = pulp.LpVariable.dict('f_%s', managed_topics, lowBound=0)

    # cost data
    cost = dict(zip(managed_topics, (DBM.DBMParam.get_message_size_in_bytes(topic) for topic in managed_topics)))

    # Create the objective
    #model += sum(cost[i] * f[i] for i in managed_topics)
    for topic in managed_topics:
        model += f[topic]

    # Constraints:
    # The total bandwidth should be lower than max bandwidth available
    model += sum(cost[i] * f[i] for i in managed_topics) <= available_bandwidth_in_bytes

    # The frequency should be higher than minimum frequency
    # Calculating the minimum frequency in relation to the priority
    fmin = dict(zip(managed_topics,
                (((DBM.DBMParam.get_max_frequency(topic) - DBM.DBMParam.get_min_frequency(topic)) * priorities[topic] + DBM.DBMParam.get_min_frequency(topic))
                for topic in managed_topics)))

    for topic in managed_topics:
            model += f[topic] >= fmin[topic]

    # The frequency should be lower than maximum frequency
    fmax = dict(zip(managed_topics,
        (DBM.DBMParam.get_max_frequency(topic) for topic in managed_topics)))
    for topic in managed_topics:
        model += f[topic] <= fmax[topic]

    # Problem is then solved with the default solver
    model.solve()

    # Create the result dictionary
    result = {}
    if model.status == 1:
        for topic in managed_topics:
            result[topic] = f[topic].value()

    return result

def optimize(managed_topics, priorities):
    available_bandwidth_in_mbits = DBM.DBMParam.get_max_bandwidth_in_mbits() * DBM.DBMParam.get_max_bandwidth_utilization()
    available_bandwidth_in_bytes = available_bandwidth_in_mbits * 125000

    if priorities_are_equal(priorities):
        result = split_bandwidth_equally(managed_topics, available_bandwidth_in_bytes)
    else:
        result = solve_optimization(managed_topics, priorities, available_bandwidth_in_bytes)

    rospy.loginfo("Available bandwidth %s KBps" % (available_bandwidth_in_bytes / 1000))
    for topic in result:
        rospy.loginfo("Topic: %s [min_freq: %sHz, max_freq: %sHz, cur_freq: %sHz, message_size: %sKB, consumed_bandw: %sKBps]"
            %(topic,
                DBM.DBMParam.get_min_frequency(topic),
                DBM.DBMParam.get_max_frequency(topic),
                result[topic],
                DBM.DBMParam.get_message_size_in_bytes(topic) / 1024,
                DBM.DBMParam.get_message_size_in_bytes(topic) * result[topic] / 1000))

    return result
        
if __name__ == '__main__':
    try:
        rospy.init_node('default_optimizer_node', anonymous=True)
        optimizer = DBM.DBMOptimizer(optimize)
        optimizer.start()

    except rospy.ROSInterruptException: pass