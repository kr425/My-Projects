
# coding: utf-8

# In[35]:

import numpy as np
from sklearn.linear_model import LogisticRegression
from sklearn.model_selection import train_test_split

data = np.genfromtxt('final595data.csv', delimiter=',', dtype=str)

X = np.transpose(np.array(data[1:,1:], dtype=float))
Y = np.transpose(np.array(data[0, 1:]))

X_train, X_test, Y_train, Y_test = train_test_split(X, Y, test_size=0.20, random_state=42)

LR = LogisticRegression(penalty='l2', dual=False, tol=0.0001, C=1.0, fit_intercept=True, intercept_scaling=1, class_weight=None, random_state=None, solver='liblinear', max_iter=100, multi_class='ovr', verbose=0, warm_start=False, n_jobs=1)
LR.fit(X_train, Y_train)
LR.score(X_test, Y_test)


# In[ ]:



