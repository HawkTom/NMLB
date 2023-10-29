#
# Lower bound for DCARP instance without pruning strategy
#
import networkx as nx
from lxml import etree as ET
import math
import time, sys
import warnings

mapname = sys.argv[1]

xml_path = "xml/{}.xml".format(mapname)
# xml_path = "toyxml/{}.xml".format(mapname)

tree = ET.parse(xml_path)
root = tree.getroot()

state = root[0]
tasks = root[1]
arcs = root[2]
info = root.find(f".//Info")

capacity = int(info.find(f".//capacity").attrib["num"])
task_num = int(root[1].attrib["num"])/2

ov = []
total_rq = 0
for vehicle in state:
    sp = int(vehicle.attrib['stop'])
    if sp == 1:
        continue
    rq = int(vehicle.attrib['rcapacity'])
    ov.append({"sp":sp, "rq":rq})
    total_rq += rq

num_ov = len(ov)

start_time = time.time()
# construct G
G = nx.Graph()
for arc in arcs:
    head = int(arc.attrib['head_node']) 
    tail = int(arc.attrib['tail_node'])
    if head == 0 and tail == 0:
        continue
    cost = int(arc.attrib['trav_cost'])
    if not G.has_node(head):
        G.add_node(head)

    if not G.has_node(tail):
        G.add_node(tail)

    if not G.has_edge(head, tail):
        G.add_edge(head, tail, weight=cost)

# dis = nx.shortest_path_length(G, n1, n2, weight="weight")

# construct GR
Gr = nx.Graph()
SC = 0
W = 0
for task in tasks:
    head = int(task.attrib['head_node']) 
    tail = int(task.attrib['tail_node'])
    if head == 1 and tail == 1:
        continue
    
    cost = int(task.attrib['serv_cost'])
    demand = int(task.attrib['demand'])
    if not Gr.has_node(head):
        Gr.add_node(head)

    if not Gr.has_node(tail):
        Gr.add_node(tail)
    
    if not Gr.has_edge(head, tail):
        Gr.add_edge(head, tail, weight=cost)
        SC += cost
        W += demand

#The minimal number of vehicles required
k = math.ceil((W - total_rq)/capacity)
k = max(k, 0)
# print(SC, W, k, num_ov, total_rq, capacity)
if k < 0:
    print("It doesn't require the new vehicles")
else:

    least_degree = {}
    setB = []
    for node in Gr.nodes:
        least_degree[node] = 0
        setB.extend([node]*Gr.degree(node))

    # Construct B'
    # Step1: sort nodes according to their distance to the depot and selet previous 2(k+num_ov) construc a set B'
    sorted_B_nodes = sorted(setB, key=lambda x:nx.shortest_path_length(G, 1, x, weight="weight"))
    setBB = sorted_B_nodes[0:2*(k+num_ov)]
    # print("BB", setBB, "\n\n")

    for i in range(num_ov):
        node = node=ov[i]["sp"]
        sorted_B_nodes = sorted(setB, key=lambda x:nx.shortest_path_length(G, node, x, weight="weight"))
        for j in range(num_ov):
            if sorted_B_nodes[j] not in setBB:
                setBB.append(sorted_B_nodes[j])
    # print("BB", setBB, "\n\n")
    
    
    for node in setBB:
        least_degree[node] += 1

    # construct augment graph
    # SetA: 2*k depot
    # SetB: len(ov) depot
    # SetC: {sp1, sp2, ..., sp_ov}
    # SetD: {node*degree, ..., node*degree}
    Gx = nx.Graph()
    node_name_dict = {}
    count = -1
    for i in range(2*k):
        count += 1
        Gx.add_node(count, node=1, set="A")
    # print("A:", count)
    
    for i in range(len(ov)):
        count += 1
        Gx.add_node(count, node=1, set="C")
    # print("C:", count)
    
    for i in range(len(ov)):
        count += 1
        Gx.add_node(count, node=ov[i]["sp"], set="D")
    # print("D:", count)

    # print(least_degree)
    for node in Gr.nodes:
        if least_degree[node] > 0:
            for i in range(least_degree[node]):
                count += 1
                Gx.add_node(count, node=node, set="B")
        if(Gr.degree(node) - least_degree[node]) % 2 == 1:
            count += 1
            Gx.add_node(count, node=node, set="B")
            # print("add", node)
        # for i in range(Gr.degree(node)):
        #     count += 1
        #     Gx.add_node(count, node=node, set="B")
    # print("B:", count)


    for i in range(count+1):
        for j in range(i+1, count+1):
            pair =  {Gx.nodes[i]["set"], Gx.nodes[j]["set"]}
            if pair == {"A", "B"} or pair == {"C", "B"} or pair == {"D", "B"} or pair == {"C", "D"}:
                dis = nx.shortest_path_length(G, Gx.nodes[i]["node"], Gx.nodes[j]["node"], weight="weight")
                Gx.add_edge(i, j, weight=dis)
            
            if pair == {"B"}:
                dis = nx.shortest_path_length(G, Gx.nodes[i]["node"], Gx.nodes[j]["node"], weight="weight")
                Gx.add_edge(i, j, weight=dis)

# for e in Gx.edges():
#     print(Gx.nodes[e[0]]["node"], Gx.nodes[e[1]]["node"])
# for node in Gr.nodes:
#     print("{0}: {1}".format(node, Gr.degree(node)))

mcpm = nx.min_weight_matching(Gx)
cc = 0

setss = {"AB":[], "BC":[], "BD":[], "CD":[], "BB":[]}
for e in mcpm:
    tc = Gx.edges[e[0], e[1]]["weight"]
    cc += tc

    # link = "{0}({2}) {1}({3})".format(Gx.nodes[e[0]]["node"], Gx.nodes[e[1]]["node"],Gx.nodes[e[0]]["set"], Gx.nodes[e[1]]["set"])
    # print(link)
            

lb = SC + cc
end_time = time.time()


OPT = int(root.attrib['D-opt'])
gap = (OPT - lb) / OPT
ss = "{} OV {} NV {} NT {} LB {} OPT {} GAP {:.3f}\n".format(sys.argv[1], num_ov, num_ov+k, task_num, lb, OPT, gap)


# BST = int(root.attrib['D-best'])
# gap = (BST - lb) / lb
# ss = "{} OV {} NV {} NT {} LB {} OPT {} AER {:.3f}\n".format(sys.argv[1], num_ov, num_ov+k, task_num, lb, BST, gap)

print("Elapsed: {:.3f} seconds.".format(end_time-start_time), ss)

# print("map: {0}, OV: {1}, NT: {2}".format(xml_path, num_ov, task_num))
# print('map: gdb{0} Elapsed: {1} seconds. A lower bound is: {2}, Best: {3}'.format(sys.argv[1], end_time-start_time, lb, best))

with open("resultegl.txt", "a+") as f:
    f.write(ss)