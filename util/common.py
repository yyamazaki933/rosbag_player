#!/usr/bin/env python3

import subprocess
import re


def execCmd(cmd):
    print("[INFO] execCmd():", cmd)

    resp = subprocess.run(
        cmd, shell=True, executable='/bin/bash', capture_output=True, text=True, timeout=3)

    return resp


def getNodeList(ros_distro):
    node_l = []

    cmd = 'source /opt/ros/' + ros_distro + '/setup.bash'
    cmd += ' && '
    cmd += 'rosnode list'

    resp = execCmd(cmd)
    nodes = resp.stdout.split('\n')

    for item in nodes:
        if item == '':
            continue
        node_l.append(item)
    return node_l


def getTopicList(ros_distro):
    topic_l = []

    cmd = 'source /opt/ros/' + ros_distro + '/setup.bash'
    cmd += ' && '
    cmd += 'rostopic list'

    resp = execCmd(cmd)
    topics = resp.stdout.split('\n')

    for item in topics:
        if item == '' or item == "/clock" or item == "/parameter_events" or item == "/rosout":
            continue
        topic_l.append(item)
    return topic_l


def getNameSpaceList(src_list):
    ns_l = []
    ns_l.append('/')

    for item in src_list:
        ns_vec = item.split('/')
        ns = ns_vec[1]
        if len(ns_vec) > 3:
            if ns in ns_l:
                continue
            ns_l.append(ns)
    return ns_l


def getNodeInfo(ros_distro, node_name):
    cmd = 'source /opt/ros/' + ros_distro + '/setup.bash'
    cmd += ' && '
    cmd += 'rosnode info ' + node_name

    resp = execCmd(cmd)
    lines = resp.stdout.split('\n')

    sub_topics = []
    pub_topics = []

    pub_node_parse = False
    sub_node_parse = False

    for line in lines:
        if line == '':
            continue

        if 'Subscribers:' in line:
            sub_node_parse = False
            pub_node_parse = True
            continue

        if 'Publishers:' in line:
            pub_node_parse = False
            sub_node_parse = True
            continue

        if 'Service Servers:' in line:
            pub_node_parse = False
            sub_node_parse = False
            continue

        if pub_node_parse:
            topic_name = line[4:].split(':')[0]
            if topic_name == "/clock" or topic_name == "/parameter_events" or topic_name == "/rosout":
                continue
            sub_topics.append(topic_name)

        if sub_node_parse:
            topic_name = line[4:].split(':')[0]
            if topic_name == "/clock" or topic_name == "/parameter_events" or topic_name == "/rosout":
                continue
            pub_topics.append(topic_name)

    return sub_topics, pub_topics


def getTopicInfo(ros_distro, topic_name):
    cmd = 'source /opt/ros/' + ros_distro + '/setup.bash'
    cmd += ' && '
    cmd += 'rostopic info ' + topic_name

    resp = execCmd(cmd)
    lines = resp.stdout.split('\n')

    pub_nodes = []
    sub_nodes = []
    pub_node_parse = False
    sub_node_parse = False
    topic_type = ''
    name = ''
    namespace = ''
    for line in lines:
        if line == '':
            continue

        if 'Type:' in line:
            topic_type = line[6:]

        if 'Publisher count:' in line:
            sub_node_parse = False
            pub_node_parse = True

        if 'Subscription count:' in line:
            pub_node_parse = False
            sub_node_parse = True

        if 'Node name:' in line:
            name = line[11:]

        if 'Node namespace:' in line:
            namespace = line[16:]

            if namespace == '/':
                node_name = '/' + name
            else:
                node_name = namespace + '/' + name

            if pub_node_parse:
                pub_nodes.append(node_name)

            if sub_node_parse:
                sub_nodes.append(node_name)

    return topic_type, pub_nodes, sub_nodes


def getTopicType(ros_distro, topic_name):
    cmd = 'source /opt/ros/' + ros_distro + '/setup.bash'
    cmd += ' && '
    cmd += 'rostopic type ' + topic_name

    resp = execCmd(cmd)
    return resp.stdout.strip('\n')


def getRosbagInfo(ros_distro, bagdir):
    cmd = 'source /opt/ros/' + ros_distro + '/setup.bash'
    cmd += ' && '
    cmd += 'rosbag info ' + bagdir

    resp = execCmd(cmd)

    if resp.stdout != '':
        info = ''
        dur = 0
        lines = resp.stdout.split('\n')

        for line in lines:
            if line == '':
                continue
            info += line + '\n'

            if 'duration:' in line:
                dur = int(re.split(r'[(s]', line)[2])

        return True, info, dur

    else:
        return False, resp.stderr, 0


def kill_proc(keyword):
    cmd = 'ps -A -f | grep ros'
    resp = execCmd(cmd)

    lines = resp.stdout.split('\n')

    for item in lines:
        if item == '':
            continue

        item_vec = item.split()
        pid = item_vec[1]
        pid = format(pid, '>10')
        proc = str.join(' ', item_vec[7:])

        if keyword in proc:
            cmd = 'kill -9 ' + pid
            resp = execCmd(cmd)
