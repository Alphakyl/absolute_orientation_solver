import copy

listoflists = []
a_list = []
for i in range(0,10):
    a_list.append(i)
    if len(a_list)>3:
        a_list.remove(a_list[0])
        listoflists.append((list(a_list), a_list[0]))
print listoflists


V_leica_prism_robot = [[None]*3 for i in range(3)]


print V_leica_prism_robot


current_base = "UGV"
# List of base dictionaries
AVAILABLE_BASE = {
    "UGV": {
        # "Vrq1"      : [-0.0713284, 0.0768284, 0.0209],
        # "Vrq2"      : [0.0713284, -0.0768284, 0.0209],
        # "Vrq3"      : [-0.0713284, -0.0768284, 0.0209],
        "Vrq1"      : [-0.0713284, 0.0768284, 0.0],
        "Vrq2"      : [0.0713284, -0.0768284, 0.0],
        "Vrq3"      : [-0.0713284, -0.0768284, 0.0],
    },
    "UAV": {
        "Vrq1"      : [-0.25, -.1, -.205],
        "Vrq2"      : [0.25,0.1, -.205],
        "Vrq3"      : [0.25,-.1, -.205],
    },
    "oldUGV": {
        "Vrq1"      : [-0.045, 0.100025, -0.0045],
        "Vrq2"      : [0.045, -0.100025, -0.0045],
        "Vrq3"      : [-0.045, -0.100025, -0.0045],
    }
}

V_robot_prism = [AVAILABLE_BASE[current_base]['Vrq1'], AVAILABLE_BASE[current_base]['Vrq2'], AVAILABLE_BASE[current_base]['Vrq3']]
print V_robot_prism
V_robot_prism[0][2] += 1
print V_robot_prism
V_robot_prism = [AVAILABLE_BASE[current_base]['Vrq1'], AVAILABLE_BASE[current_base]['Vrq2'], AVAILABLE_BASE[current_base]['Vrq3']]
print V_robot_prism

print AVAILABLE_BASE["UGV"]["Vrq1"]

print '+++++++++++++++++++++++++++++++++++++++++++++++++++++++'
AVAILABLE_BASE = {
    "UGV": {
        # "Vrq1"      : [-0.0713284, 0.0768284, 0.0209],
        # "Vrq2"      : [0.0713284, -0.0768284, 0.0209],
        # "Vrq3"      : [-0.0713284, -0.0768284, 0.0209],
        "Vrq1"      : [-0.0713284, 0.0768284, 0.0],
        "Vrq2"      : [0.0713284, -0.0768284, 0.0],
        "Vrq3"      : [-0.0713284, -0.0768284, 0.0],
    },
    "UAV": {
        "Vrq1"      : [-0.25, -.1, -.205],
        "Vrq2"      : [0.25,0.1, -.205],
        "Vrq3"      : [0.25,-.1, -.205],
    },
    "oldUGV": {
        "Vrq1"      : [-0.045, 0.100025, -0.0045],
        "Vrq2"      : [0.045, -0.100025, -0.0045],
        "Vrq3"      : [-0.045, -0.100025, -0.0045],
    }
}
V_robot_prism = [copy.deepcopy(AVAILABLE_BASE['UGV']['Vrq1']), copy.deepcopy(AVAILABLE_BASE['UGV']['Vrq2']), copy.deepcopy(AVAILABLE_BASE['UGV']['Vrq3'])]

print V_robot_prism
V_robot_prism[0][2] += 1
print V_robot_prism
print AVAILABLE_BASE["UGV"]["Vrq1"]

