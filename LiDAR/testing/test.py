import yaml
import time 
import numpy as np
import controller as contr
import interlock as intr

with open('lidar_data.yaml', 'r') as stream:
    rows_to_data_lists = yaml.safe_load(stream)

data = []
for data_list in rows_to_data_lists.values():
    data.extend(data_list)
data = np.array(data)

start = time.time()
cert = contr.controller(data)
end = time.time()
print('Controller said', cert.action)
print(str(end - start) + "s to call controller")

start = time.time()
result = intr.interlock(cert)
end = time.time()
print('Interlock said', result)
print(str(end - start) + "s to call interlock")