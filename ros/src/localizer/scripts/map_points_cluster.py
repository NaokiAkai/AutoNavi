#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import re
import numpy as np
from matplotlib import pyplot as plt
from scipy.linalg import cholesky
from numpy.random import *
import math
from scipy.special import gamma

data_dim = 2 # データの次元数（x，y座標なので２）
iteration_num = 1000 # 反復回数（１つの点を選択して）


# ハイパーパラメータ（それぞれのパラメータの意味を理解すること）
alpha = 1.0 # 新しいクラスの生成に関するパラメータ（大きいほど新しいクラスタが生成されやすい）
S = np.array([[10.0, 0.0], [0.0, 0.000000001]]) # 
nu = 3.0 # 
beta = 0.1 # 
mu0 = np.zeros(data_dim) # データの平均値に関するパラメータ（データセットの平均値として定義する）


# あらかじめPCLでクラスタリングした結果を初期値として用いる（fdl/src/fdl/src/map_points_clusterで作成している）
# 最初にすべての点の数と初期のクラス数を数える
data_dir = '/tmp/clustered_points/'
files = os.listdir(data_dir)
data_num = 0
class_num = 0
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
class_num = count
print data_num, class_num

# データ保管用の配列を定義してデータを代入
X = np.empty((data_num, data_dim)) # データ用の配列
C = np.empty((data_num), np.int32) # データに割り振るクラスの用配列
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
				C[i] = count
				mu0[0] += float(data[0])
				mu0[1] += float(data[1])
				i += 1
			elif data[0] == 'DATA':
				add_point = True
		fp.close()
		count += 1
#mu0 /= data_num
mu0[0] = mu0[1] = 0.0
print X, X.shape
print C, C.shape












# ここからが階層型ディリクレ過程を用いたクラスタリングに関するところ

# c番目のクラスタを削除する
def delete_target_cluster(c, ns, mus, covs, lambdas, Sqs):
	global class_num
	ns = np.delete(ns, c)
	mus = np.delete(mus, c, 0)
	covs = np.delete(covs, c, 0)
	lambdas = np.delete(lambdas, c, 0)
	Sqs = np.delete(Sqs, c, 0)
	class_num -= 1
	# 削除したクラスより大きいクラスには，1つ小さい値のクラスを割り振る
	for i in range(data_num):
		if C[i] >= c:
			C[i] -= 1
	return ns, mus, covs, lambdas, Sqs


# データの含まれていないクラスタを削除する
def delete_empty_clusters(ns, mus, covs, lambdas, Sqs):
	global class_num
	target_index = 0
	for i in range(class_num):
		if ns[target_index] == 0:
			ns, mus, covs, lambdas, Sqs = delete_target_cluster(target_index, ns, mus, covs, lambdas, Sqs)
			target_index -= 1
		target_index += 1
		if target_index >= ns.size:
			break
	return ns, mus, covs, lambdas, Sqs


# c番目のクラスタのパラメータを計算する
def compute_target_cluster_parameters(c):
	n = 0 # c番目のクラスタのデータ数
	xs = np.empty(data_dim) # c番目のクラスタのデータ
	# c番目のクラスタのデータを取得
	for i in range(data_num):
		if C[i] == c:
			if n == 0:
				xs = X[i]
			else:
				xs = np.append(xs, X[i], axis=0)
			n += 1
	xs = np.reshape(xs, (n, data_dim))
	mu = np.mean(xs, axis=0) # axis=0の場合は列方向の要素の平均を求める
	cov = np.zeros((data_dim, data_dim)) # クラスタの共分散行列
	for i in range(n):
		for j in range(data_dim):
			for k in range(data_dim):
				cov[j][k] += (xs[i][j] - mu[j]) * (xs[i][k] - mu[k])
	lambda_ = np.zeros((data_dim, data_dim)) # クラスタの共分散の逆行列
	if n <= 1:
		cov = np.zeros((data_dim, data_dim)) # データが1つ以下の場合分散はゼロとする
		lambda_ = cov
	else:
		cov /= n - 1
		det = np.linalg.det(cov)
		det2 = det * det
		if det2 < 0.000000001:
			lambda_ = np.zeros((data_dim, data_dim)) # 逆行列が計算できないのでとりあえずゼロにする
		else:
			lambda_ = np.linalg.inv(cov)
	Sq_inv = np.zeros((data_dim, data_dim))
	for i in range(n):
		diff = xs[i] - mu
		for j in range(data_dim):
			for k in range(data_dim):
				Sq_inv[j][k] += diff[j] * diff[k]
	tmp = np.zeros((data_dim, data_dim))
	diff = mu - mu0
	for i in range(data_dim):
		for j in range(data_dim):
			tmp[i][j] += diff[i] * diff[j]
	Sq_inv += np.linalg.inv(S) + float(n) * beta / ((float(n) + beta)) * tmp
	Sq = np.linalg.inv(Sq_inv)
	return n, mu, cov, lambda_, Sq


# クラスタのパラメータとmu_paramsとlambda_paramsを更新するのに必要なパラメータを計算する
# 途中で空のクラスタは削除する
def compute_all_cluster_parameters():
	ns = np.zeros((class_num), np.int32) # クラスタのデータ数
	mus = np.zeros((class_num, data_dim)) # クラスタの平均
	covs = np.zeros((class_num, data_dim, data_dim)) # クラスタの共分散行列
	lambdas = np.zeros((class_num, data_dim, data_dim)) # クラスタの共分散の逆行列
	Sqs = np.zeros((class_num, data_dim, data_dim)) # これは何て言うの？（更新に必要なパラメータ）
	# クラスタの平均を計算
	for i in range(data_num):
		ns[C[i]] += 1
		mus[C[i]] += X[i]
	for i in range(class_num):
		if ns[i] == 0:
			mus[i] = np.zeros(data_dim) # データが含まれないクラスタの平均はゼロとする
		else:
			mus[i] /= ns[i]
	# データが含まれないクラスタを削除する
	ns, mus, covs, lambdas, Sqs = delete_empty_clusters(ns, mus, covs, lambdas, Sqs)
	# 共分散を計算
	for i in range(data_num):
		for j in range(data_dim):
			for k in range(data_dim):
				covs[C[i]][j][k] += (X[i][j] - mus[C[i]][j]) * (X[i][k] - mus[C[i]][k])
	for i in range(class_num):
		if ns[i] <= 1:
			covs[i] = np.zeros((data_dim, data_dim)) # データが1つ以下のクラスタの分散はゼロとする
			lambdas[i] = covs[i]
		else:
			covs[i] /= ns[i] - 1
			det = np.linalg.det(covs[i])
			det2 = det * det
			if det2 < 0.000000001:
				lambdas[i] = np.zeros((data_dim, data_dim)) # 逆行列が計算できないのでとりあえずゼロにする
			else:
				lambdas[i] = np.linalg.inv(covs[i])
	# Sqの計算
	Sq_invs = np.zeros((class_num, data_dim, data_dim))
	for i in range(data_num):
		diff = X[i] - mus[C[i]]
		for j in range(data_dim):
			for k in range(data_dim):
				Sq_invs[C[i]][j][k] += diff[j] * diff[k]
	for i in range(class_num):
		tmp = np.zeros((data_dim, data_dim))
		diff = mus[C[i]] - mu0
		for j in range(data_dim):
			for k in range(data_dim):
				tmp[j][k] += diff[j] * diff[k]
		Sq_invs[i] += np.linalg.inv(S) + float(ns[C[i]]) * beta / (float(ns[C[i]]) + beta) * tmp
		Sqs[i] = np.linalg.inv(Sq_invs[i])
	return ns, mus, covs, lambdas, Sqs


# c番目のクラスのハイパーパラメータを計算する
def compute_target_hyper_parameters(c, n, mu, Sq):
	muc = (float(n) * mu + beta * mu0) / (float(n) + beta)
	nuc = nu + float(n)
	lambdac = (float(n) + beta) * nuc * Sq
	return muc, nuc, lambdac


# 最終的に求めたいハイパーパラメータを計算する
# このプログラムでは対象のデータが正規分布に基づくと仮定しているため，ハイパーパラメータは平均と共分散になる
# 計算の都合上，共分散はその逆行列（情報行列）として求めている
def compute_hyper_parameters(ns, mus, Sq):
	mucs = np.zeros((class_num, data_dim)) # 最終的に求めたいクラスタの平均値達
	nucs = np.zeros(class_num) # 情報行列を計算するときに使用するパラメータ（ハイパーパラメータではない）
	lambdacs = np.zeros((class_num, data_dim, data_dim))  # 最終的に求めたいクラスタの共分散の逆行列達
	for i in range(class_num):
		mucs[i], nucs[i], lambdacs[i] = compute_target_hyper_parameters(i, ns[i], mus[i], Sqs[i])
	return mucs, nucs, lambdacs


# ランダムにデータを取り出し，トータルデータ数とクラスタのデータ数を1つ減らす
# 取り出したデータとそのクラスは一度データ集合から削除する
def pick_up_data_randomly(C, X, ns):
	global data_num
	target_index = randint(0, data_num)
	c = C[target_index] # 取り出したデータのクラス
	x = X[target_index] # 取り出したデータ
	C = np.delete(C, target_index)
	X = np.delete(X, target_index, 0)
	data_num -= 1
	ns[c] -= 1
	return c, x, C, X, ns


# 不要になった（消滅したクラスタの）ハイパーパラメータを削除する
def delete_hyper_parameters(c, mucs, nucs, lambdacs):
	mucs = np.delete(mucs, c, 0)
	nucs = np.delete(nucs, c)
	lambdacs = np.delete(lambdacs, c, 0)
	return mucs, nucs, lambdacs


# 与えられたパラメータの下で正規分布の値を計算する
def compute_normal_distribution_probability(x, mu, sigma):
	a = np.power((2.0 * np.pi), float(data_dim) / 2.0) * np.sqrt(np.linalg.det(sigma))
	sigma_inv = np.linalg.inv(sigma)
	diff = x - mu
	tmp = np.zeros(data_dim)
	for i in range(data_dim):
		for j in range(data_dim):
			tmp[i] += sigma_inv[i][j] * diff[j]
	d = 0.0
	for i in range(data_dim):
		d += diff[i] * tmp[i]
	p = np.exp(-0.5 * d) / a
	return p


# 与えられたパラメータの下でウィシャート分布の値を計算する
def compute_wishart_distribution_probability(lambda_, nu_, S_):
	p = np.power(np.linalg.det(lambda_), (nu_ - float(data_dim - 1)) / 2.0)
	p *= np.exp(-0.5 * np.trace(np.dot(np.linalg.inv(S_), lambda_)))
	p /= np.power(2.0, nu_ * float(data_dim) / 2.0)
	p /= np.power(np.pi, float(data_dim) * float(data_dim - 1) / 4.0)
	p /= np.power(np.linalg.det(S_), nu_ / 2.0)
	q = 1.0
	for i in range(data_dim):
		q *= gamma((nu_ + 1.0 - float(i)) / 2.0)
	p /= q
	return p


# データxがどのクラスタ，もしくは新しいクラスタに属すべきかを確率的に決定する
def probabilistically_generate_class(x, ns, mucs, lambdacs):
	p = np.zeros(class_num + 1)
	# 既存クラスタに所属する確率を計算
	for i in range(class_num):
		sigma = np.linalg.inv(lambdacs[i])
		# データを取り出したときにすでにdata_numの数を1つ少なくしているので-1はしない（参考URLではデータ数を-1しているので注意）
		k = float(ns[i]) / (float(data_num) + alpha)
		if ns[i] >= 2:
			p[i] = k * compute_normal_distribution_probability(x, mucs[i], sigma)
		else:
			p[i] = k * 0.5 # データが少なく共分散が定義されていないので，適当な確率を割り振る（これで良いの？）
	# 新しいクラスタを作成する確率の計算
	Sb_inv = np.zeros((data_dim, data_dim))
	diff = x - mu0
	for i in range(data_dim):
		for j in range(data_dim):
			Sb_inv[i][j] += diff[i] * diff[j]
	Sb_inv += np.linalg.inv(S) + beta / (1.0 + beta) * Sb_inv
	Sb = np.linalg.inv(Sb_inv)
	# データを取り出したときにすでにdata_numの数を1つ少なくしているので-1はしない（参考URLではデータ数を-1しているので注意）
	p[class_num] = alpha / (float(data_num) + alpha)
	p[class_num] *= np.power(beta / ((1.0 + beta) * np.pi), float(data_dim) / 2.0)
	p[class_num] *= np.power(np.linalg.det(Sb), (nu + 1.0) / 2.0) * gamma((nu + 1.0) / 2.0)
	p[class_num] /= np.power(np.linalg.det(Sb), nu / 2.0) * gamma((nu + 1.0 - float(data_dim)) / 2.0)
	# クラスを決定する
	p /= np.sum(p) # 正規化して確率分布とする
	r = rand() # 0~1の乱数を発生させてダーツ方式でクラスを決定する
	th = 0.0
	for i in range(class_num):
		th += p[i]
		if r < th:
			return i
	return class_num


# クラスタの分割方法，および配置に関する事後確率を計算する
def compute_posterior(ns, mucs, lambdacs):
	w = 0.0
	for i in range(class_num):
		sigma = np.linalg.inv(beta * lambdacs[i])
		p1 = np.log(compute_normal_distribution_probability(mucs[i], mu0, sigma))
		p1 += np.log(compute_wishart_distribution_probability(lambdacs[i], nu, S))
		for j in range(data_num):
			if C[j] == i:
				sigma = np.linalg.inv(lambdacs[i])
				p1 += np.log(compute_normal_distribution_probability(X[i], mucs[i], sigma))
		w += p1
	a = np.power(alpha, float(class_num))
	for i in range(class_num):
		a *= math.factorial(ns[i] - 1)
	b = 1.0
	for i in range(data_num):
		b *= (alpha + float(i))
	p = a / b * np.exp(w)
	return p


def gen_pcd_data():
	for i in range(class_num):
		point_num = 0
		for j in range(data_num):
			if (C[j] == i):
				point_num += 1
		fname = './data/clustered_points_' + str(i) + '.pcd'
		fp = open(fname, 'w')
		fp.write('# .PCD v0.7 - Point Cloud Data file format\n')
		fp.write('VERSION 0.7\n')
		fp.write('FIELDS x y z\n')
		fp.write('SIZE 4 4 4\n')
		fp.write('TYPE F F F\n')
		fp.write('COUNT 1 1 1\n')
		fp.write('WIDTH ')
		fp.write(str(point_num))
		fp.write('\n')
		fp.write('HEIGHT 1\n')
		fp.write('VIEWPOINT 0 0 0 1 0 0 0\n')
		fp.write('POINTS ')
		fp.write(str(point_num))
		fp.write('\n')
		fp.write('DATA ascii\n')
		for j in range(data_num):
			if (C[j] == i):
				fp.write(str(X[j][0]))
				fp.write(' ')
				fp.write(str(X[j][1]))
				fp.write(' ')
				fp.write('0\n')
		fp.close()


def plot_data():
	for i in range(class_num):
		Y = np.empty(data_dim)
		k = 0
		for j in range(data_num):
			if C[j] == i:
				if k == 0:
					Y = X[j]
				else:
					Y = np.append(Y, X[j], axis=0)
				k += 1
		Y = np.reshape(Y, (k, data_dim))
		plt.plot(Y[:,0], Y[:,1], 'o') # 異なる色でデータ点をプロット
	plt.show()









# クラスタリングのスタート
ns, mus, covs, lambdas, Sqs = compute_all_cluster_parameters()
mucs, nucs, lambdacs = compute_hyper_parameters(ns, mus, Sqs)
for i in range(iteration_num):
	c, x, C, X, ns = pick_up_data_randomly(C, X, ns)
	if ns[c] == 0:
		ns, mus, covs, lambdas, Sqs = delete_target_cluster(c, ns, mus, covs, lambdas, Sqs)
		mucs, nucs, lambdacs = delete_hyper_parameters(c, mucs, nucs, lambdacs)
	else:
		ns[c], mus[c], covs[c], lambdas[c], Sqs[c] = compute_target_cluster_parameters(c)
		mucs[c], nucs[c], lambdacs[c] = compute_target_hyper_parameters(c, ns[c], mus[c], Sqs[c])
	new_c = probabilistically_generate_class(x, ns, mucs, lambdacs)
	if new_c == class_num:
		class_num += 1
	ns = np.append(ns, 1)
	C = np.append(C, new_c)
	X = np.append(X, x)
	data_num += 1
	X = np.reshape(X, (data_num, data_dim))
	ns, mus, covs, lambdas, Sqs = compute_all_cluster_parameters()
	mucs, nucs, lambdacs = compute_hyper_parameters(ns, mus, Sqs)
	print 'trial_num', i, 'class_num', class_num
	# posterior = compute_posterior(ns, mucs, lambdacs)
	# print i, class_num, posterior
print mus
print covs
print class_num
gen_pcd_data()
plot_data()



