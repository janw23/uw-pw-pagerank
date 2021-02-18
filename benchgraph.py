import sys
import pathlib
import re

from matplotlib import pyplot as plt
import numpy as np

path = pathlib.Path(sys.argv[1])

reg = "^(.+) Test \[((.*) nodes, )?(.+)\].*took:\s*(.+)s"

tests = dict()

with open(path, "r") as file:
	for line in file:
		search_result = re.search(reg, line)

		test_name = search_result.group(1)
		num_nodes = search_result.group(3)
		computer = search_result.group(4)
		took = float(search_result.group(5))

		num_nodes = int(num_nodes) if num_nodes is not None else -1

		if test_name not in tests:
			tests[test_name] = dict()

		if computer not in tests[test_name]:
			tests[test_name][computer] = set()

		tests[test_name][computer].add((num_nodes, took))

for testKV in tests.items():
	test_name = testKV[0]

	computersDict = testKV[1]
	computers = [item[0] for item in computersDict.items()]
	
	data = []
	nodes = None

	for computer in computers:
		computer_data = []

		pairs = sorted(list(testKV[1][computer]))
		nodes = [n for (n, _) in pairs]

		for pair in pairs:
			num_nodes, took = pair
			computer_data.append(took)

		data.append(computer_data)

	data = np.array(data)
	print(data)

	fig, ax = plt.subplots()
	X = np.arange(len(nodes))

	width = 0.75 / len(data)

	for index in range(len(data)):
		ax.bar(X + (index - len(data) / 2 + 0.5) * width, data[index], width=width, label=computers[index])

	ax.set_ylabel("seconds")
	ax.set_title("execution times")
	ax.set_xlabel("nodes")
	ax.set_xticks(X)
	ax.set_xticklabels(nodes)
	ax.legend()

	plt.show()


