#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import re
import numpy as np
from sklearn.metrics import pairwise
import matplotlib.pyplot as plt

class KernelKMeans(object):
	def __init__(self, n_clusters=8, max_iter=300, kernel=pairwise.linear_kernel):
		self.n_clusters = n_clusters
		self.max_iter = max_iter
		self.kernel = kernel

	def _initialize_cluster(self, X):
		self.N = np.shape(X)[0]
		self.y = np.random.randint(low=0, high=self.n_clusters, size=self.N)
		self.K = self.kernel(X)

	def fit_predict(self, X):
		self._initialize_cluster(X)
		for _ in range(self.max_iter):
			obj = np.tile(np.diag(self.K).reshape((-1, 1)), self.n_clusters)
			N_c = np.bincount(self.y)
			for c in range(self.n_clusters):
				obj[:, c] -= 2 * np.sum((self.K)[:, self.y == c], axis=1) / N_c[c]
				obj[:, c] += np.sum((self.K)[self.y == c][:, self.y == c]) / (N_c[c] ** 2)
			self.y = np.argmin(obj, axis=1)
		return self.y

def make_dataset(N):
	X = X = np.zeros((N, 2))
	X[: N / 2, 0] = 10 * np.cos(np.linspace(0.2 * np.pi, N / 2, num=N / 2))
	X[N / 2:, 0] = np.random.randn(N / 2)
	X[: N / 2, 1] = 10 * np.sin(np.linspace(0.2 * np.pi, N / 2, num=N / 2))
	X[N / 2:, 1] = np.random.randn(N / 2)
	return X

def make_dataset_map():
	data_dir = '/tmp/clustered_points/'
	files = os.listdir(data_dir)
	data_num = 0
	count = 0
	for file in files:
		index = re.search('.pcd', file)
		if index:
			fname = data_dir + 'clustered_points_' + str(count) + '.pcd'
			fp = open(fname, 'r')
			for data_str in fp:
				data = data_str[:-1].split(' ')
				if data[0] == 'POINTS':
					data_num += int(data[1])
					break
			fp.close()
			count += 1
	X = np.empty((data_num, 2))
	count = 0
	i = 0
	for file in files:
		index = re.search('.pcd', file)
		if index:
			fname = data_dir + 'clustered_points_' + str(count) + '.pcd'
			fp = open(fname, 'r')
			add_point = False
			for data_str in fp:
				data = data_str[:-1].split(' ')
				if add_point == True:
					X[i][0] = float(data[0])
					X[i][1] = float(data[1])
					i += 1
				elif data[0] == 'DATA':
					add_point = True
			fp.close()
			count += 1
	return X

if __name__ == '__main__':
#	X = make_dataset(500)
	X = make_dataset_map()

	# kernel k-means with linear kernel
	kkm_linear = KernelKMeans(n_clusters=2, max_iter=100, kernel=pairwise.linear_kernel)
	y_linear = kkm_linear.fit_predict(X)

	# kernel k-means with rbf kernel
	kkm_rbf = KernelKMeans(n_clusters=2, max_iter=100, kernel=lambda X: pairwise.rbf_kernel(X, gamma=0.1))
	y_rbf = kkm_rbf.fit_predict(X)
	plt.subplot(121)
	plt.scatter(X[y_linear == 0][:, 0], X[y_linear == 0][:, 1], c="blue")
	plt.scatter(X[y_linear == 1][:, 0], X[y_linear == 1][:, 1], c="red")
	plt.title("linear kernel")
	plt.axis("scaled")
	plt.subplot(122)
	plt.scatter(X[y_rbf == 0][:, 0], X[y_rbf == 0][:, 1], c="blue")
	plt.scatter(X[y_rbf == 1][:, 0], X[y_rbf == 1][:, 1], c="red")
	plt.title("rbf kernel")
	plt.axis("scaled")
	plt.show()
