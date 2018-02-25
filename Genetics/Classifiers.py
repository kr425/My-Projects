# -*- coding: utf-8 -*-
"""
Created on Tue May 30 19:49:36 2017

@author: Krlashbrook 595 Final Project
"""

import numpy as np
from numpy import *
import scipy.linalg as la
import numpy.matlib as M
import matplotlib.pyplot as plt
import numpy.random
from numpy.matlib import rand,zeros,ones,empty,eye
from sklearn.datasets import load_iris
from sklearn import datasets
from sklearn.multiclass import OneVsRestClassifier
from sklearn.svm import LinearSVC
from sklearn.model_selection import train_test_split
from sklearn.model_selection import GridSearchCV
from sklearn import svm


data = np.genfromtxt('final595data.csv', delimiter=',', dtype=str)
X = np.transpose(np.array(data[1:,1:]))
Y = np.transpose(np.array(data[0, 1:]))

X_train, X_test,y_train,y_test= train_test_split(X,Y, test_size=0.2, random_state=0)
#y_train.astype(float)
X_test.astype(float)
#y_test.astype(float)

#######used to save test and train data for use in orange#############
#==============================================================================
# np.savetxt("Xtrain.csv", X_train, delimiter=",",fmt='%s')
# np.savetxt("ytrain.csv", y_train, delimiter=",",fmt='%s')
# np.savetxt("Xtest.csv", X_test, delimiter=",",fmt='%s')
# np.savetxt("ytest.csv", y_test, delimiter=",",fmt='%s')
#==============================================================================

from sklearn.naive_bayes import GaussianNB
gnb = GaussianNB()
y_pred = gnb.fit(X_train,y_train).predict(X_test)
print(gnb.score(X_test, y_test))

from sklearn.naive_bayes import MultinomialNB
clf = MultinomialNB()
clf.fit(X_train,y_train).predict(X_test)
print(clf.score(X_test, y_test))
#==============================================================================
# 
# from sklearn.naive_bayes import BernoulliNB
# clfb = BernoulliNB()
# clfb.fit(X_train,y_train).predict(X_test)
# print(clfb.score(X_test, y_test))
#==============================================================================

from sklearn import tree
from sklearn.ensemble import RandomForestClassifier
#clft = tree.DecisionTreeClassifier(max_depth=50)
print("test")
clfrt=RandomForestClassifier(max_depth=80, n_estimators=30, max_features=21)
print("test")
param_grid = {"max_depth":[1,5,10,100],"n_estimators":[1,5,10,100]}
print("test")
clf_= GridSearchCV(RandomForestClassifier(),param_grid=param_grid, cv=10)
print("test")
clf_.fit(X_train, y_train)
print("test")
print(clf_.score(X_test, y_test))

clft = clft.fit(X_train,y_train)
print(clft.score(X_test, y_test))
clfrt = clfrt.fit(X_train,y_train)
print(clfrt.score(X_test, y_test))
clf_ = svm.SVC(degree=1,kernel='rbf',max_iter=100,tol=2)
param_grid = {"degree":range(1,10),"max_iter":[-1,1,3,5,8,10,13,15,20,25,100,500,1000],"tol":[.1,.01,.001,.00001,.00000001,.00000000001]}
clf_= GridSearchCV(clf,param_grid=param_grid, cv=10)
clf_.fit(X_train, y_train)  
print(clf_.score(X_test, y_test))