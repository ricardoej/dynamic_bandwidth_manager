#!/usr/bin/env python

import rospy
import dynamic_bandwidth_manager
import pulp
import numpy

def start(managed_topics):
    # Initialize the model
    model = pulp.LpProblem("Dynamic Bandwidth Optmization for Multirobot Systems", pulp.LpMaximize)

    # Make a list of managed topics (communication channel)
    managed_topics = np.asarray(managed_topics)

    # Create a dictionary of pulp variables with keys from managed topics
    # the default lower bound is -inf
    f = pulp.LpVariable.dict('f_%s', managed_topics, lowBound=0)

    # cost data
    cost = dict(zip(managed_topics, (DBMParam.getMessageSizeInBytes(topic) for topic in managed_topics)))

    # Create the objective
    model += sum(cost[i] * f[i] for i in managed_topics)

    # Constraints:
    # The total bandwidth should be lower than max bandwidth available
    model += sum(cost[i] * f[i] for i in managed_topics) <= DBMParam.getMaxBandwidthInMBits() * 1310720 * DBMParam.getMaxBandwidthUtilization()

    # The frequency should be higher than minimum frequency
    # Calculating the minimum frequency in relation to the priority
    fmin = dict(zip(managed_topics,
    (((DBMParam.getMaxFrequency(topic) - DBMParam.getMinFrequency(topic)) * DBMParam.getPriority(topic) + DBMParam.getMinFrequency(topic))
    for topic in managed_topics)))
    for topic in managed_topics:
    model += f[topic] >= fmin[topic]

    # The frequency should be lower than maximum frequency
    for topic in managed_topics:
    model += f[topic] <= DBMParam.getMaxFrequency(topic)

    # Write model in file model.lp
    model.writeLP("model.lp")

    # Problem is then solved with the default solver
    model.solve()

    # Print the result
    for topic in managed_topics:
        print 'The frequency of %s is %s (status: %s; fmin: %s)' %(topic, f[topic].value(), model.status, fmin)

    frequencies = {}
    if model.status == 1:
        for topic in managed_topics:
            frequencies[topic] = f[topic].value())

    return frequencies

if __name__ == '__main__':
    try:
        rospy.init_node('dbm_optimizer_node', anonymous=True)
    except rospy.ROSInterruptException: pass