
# coding: utf-8

# In[13]:

import numpy as np
import pandas as pd

f = open('595_data.txt')
#f = open('595dataaa')
data = []
labels = []
count = 0
for line in f:
    if line[:30] == "!Sample_characteristics_ch1\t\"l":
        labels = line.replace('"',"").strip().split("\t")
    if line[0] == '!' or line[0] == '\n':
        continue
    else:
        data.append(line.replace('"',"").strip().split("\t"))


# In[14]:

print (labels[1:])
print (len(labels[1:]))


# In[15]:

for j in range(2097):
    if('B-ALL' in labels[j] or 'ALL with t(12;21)' in labels[j] or 'ALL with t(1;19)' in labels[j] or 'ALL with hyperdiploid karyotype' in labels[j]):
        labels[j]='B-ALL'
    elif('T-ALL' in labels[j]):
        labels[j]='T-ALL'
    elif('AML' in labels[j]):
        labels[j]='AML'


# In[94]:

# Calculate p-values for each probe
from scipy.stats import ttest_ind
p_val_b_all = []
for i in range(1,54614):
    #print i 
    temp = []
    non = []
    for j in range(1,len(labels)):
        if labels[j] == 'B-ALL':
            temp.append(float(data[i][j]))
        else:
            non.append(float(data[i][j]))
            
    p_val_b_all.append(ttest_ind(temp, non, equal_var=False)[1])

p_val_t_all = []
for i in range(1,54614):
    #print i 
    temp = []
    non = []
    for j in range(1,len(labels)):
        if labels[j] == 'T-ALL':
            temp.append(float(data[i][j]))
        else:
            non.append(float(data[i][j]))
            
    p_val_t_all.append(ttest_ind(temp, non, equal_var=False)[1])

p_val_aml = []
for i in range(1,54614):
    #print i 
    temp = []
    non = []
    for j in range(1,len(labels)):
        if labels[j] == 'AML':
            temp.append(float(data[i][j]))
        else:
            non.append(float(data[i][j]))
            
    p_val_aml.append(ttest_ind(temp, non, equal_var=False)[1])

p_val_mds = []
for i in range(1,54614):
    #print i 
    temp = []
    non = []
    for j in range(1,len(labels)):
        if labels[j] == 'leukemia class: MDS':
            temp.append(float(data[i][j]))
        else:
            non.append(float(data[i][j]))
            
    p_val_mds.append(ttest_ind(temp, non, equal_var=False)[1])
    
p_val_cml = []
for i in range(1,54614):
    #print i 
    temp = []
    non = []
    for j in range(1,len(labels)):
        if labels[j] == 'leukemia class: CML':
            temp.append(float(data[i][j]))
        else:
            non.append(float(data[i][j]))
            
    p_val_cml.append(ttest_ind(temp, non, equal_var=False)[1])

p_val_cll = []
for i in range(1,54614):
    #print i 
    temp = []
    non = []
    for j in range(1,len(labels)):
        if labels[j] == 'leukemia class: CLL':
            temp.append(float(data[i][j]))
        else:
            non.append(float(data[i][j]))
            
    p_val_cll.append(ttest_ind(temp, non, equal_var=False)[1])
    
    
p_val_healthy = []
for i in range(1,54614):
    #print i 
    temp = []
    non = []
    for j in range(1,len(labels)):
        if labels[j] == 'leukemia class: Non-leukemia and healthy bone marrow':
            temp.append(float(data[i][j]))
        else:
            non.append(float(data[i][j]))
            
    p_val_healthy.append(ttest_ind(temp, non, equal_var=False)[1])


# In[44]:

# Minimum 10 p-values for each class
#==============================================================================
# k = 10
# arr_b_all = np.array(p_val_b_all)
# print "B-all"
# print arr_b_all[arr_b_all.argsort()[:k]]
# print arr_b_all.argsort()[:k]
# 
# arr_t_all = np.array(p_val_t_all)
# print "T-all"
# print arr_t_all[arr_t_all.argsort()[:k]]
# print arr_t_all.argsort()[:k]
# 
# print "AML"
# arr_aml = np.array(p_val_aml)
# print arr_aml[arr_aml.argsort()[:k]]
# print arr_aml.argsort()[:k]
# 
# print "MDS"
# arr_mds = np.array(p_val_mds)
# print arr_mds[arr_mds.argsort()[:k]]
# print arr_mds.argsort()[:k]
# 
# print "CML"
# arr_cml = np.array(p_val_cml)
# print arr_cml[arr_cml.argsort()[:k]]
# print arr_cml.argsort()[:k]
# 
# print "CLL"
# arr_cll = np.array(p_val_cll)
# print arr_cll[arr_cll.argsort()[:k]]
# print arr_cll.argsort()[:k]
# 
# print "Healthy"
# arr_healthy = np.array(p_val_healthy)
# print arr_healthy[arr_healthy.argsort()[:k]]
# print arr_healthy.argsort()[:k]
# 
#==============================================================================

# In[113]:

import math
# All p-values less than 10^-10 for each class
arr_b_all = np.array(p_val_b_all)
b_all_row_indices = []
b_all_row_p = []
print ("B-all")
for i in range(len(p_val_b_all)):
    if p_val_b_all[i] <= math.pow(10, -10):
        b_all_row_indices.append(i+1)  
        b_all_row_p.append(p_val_b_all[i])
        
print (len(b_all_row_indices))
#print b_all_row_p


arr_t_all = np.array(p_val_t_all)
t_all_row_indices = []
t_all_row_p = []
print ("T-all")
for i in range(len(p_val_t_all)):
    if p_val_t_all[i] <= math.pow(10, -10):
        t_all_row_indices.append(i+1)  
        t_all_row_p.append(p_val_t_all[i])
#print len(t_all_row_indices)
#print t_all_row_p

print ("AML")
aml_row_indices = []
aml_row_p = []
arr_aml = np.array(p_val_aml)
for i in range(len(p_val_aml)):
    if p_val_aml[i] <= math.pow(10, -10):
        aml_row_indices.append(i+1)  
        aml_row_p.append(p_val_aml[i])
#print len(aml_row_indices)
#print aml_row_p

print ("MDS")
arr_mds = np.array(p_val_mds)
mds_row_indices = []
mds_row_p = []
for i in range(len(p_val_mds)):
    if p_val_mds[i] <= math.pow(10, -10):
        mds_row_indices.append(i+1)  
        mds_row_p.append(p_val_mds[i])
#print len(mds_row_indices)
#print mds_row_p

print ("CML")
arr_cml = np.array(p_val_cml)
cml_row_indices = []
cml_row_p = []
for i in range(len(p_val_cml)):
    if p_val_cml[i] <= math.pow(10, -10):
        cml_row_indices.append(i+1)  
        cml_row_p.append(p_val_cml[i])
#print len(cml_row_indices)
#print cml_row_p

print ("CLL")
arr_cll = np.array(p_val_cll)
cll_row_indices = []
cll_row_p = []
for i in range(len(p_val_cll)):
    if p_val_cll[i] <= math.pow(10, -10):
        cll_row_indices.append(i+1)  
        cll_row_p.append(p_val_cll[i])
#print len(cll_row_indices)
#print cll_row_p

print ("Healthy")
arr_healthy = np.array(p_val_healthy)
healthy_row_indices = []
healthy_row_p = []
for i in range(len(p_val_healthy)):
    if p_val_healthy[i] <= math.pow(10, -10):
        healthy_row_indices.append(i+1)  
        healthy_row_p.append(p_val_healthy[i])
#print len(healthy_row_indices)
#print healthy_row_p


# In[114]:

# Difference of means
DoM_b_all = []
for i in b_all_row_indices:
    #print i 
    temp = []
    non = []
    for j in range(1,len(labels)):
        if labels[j] == 'B-ALL':
            temp.append(float(data[i][j]))
        else:
            non.append(float(data[i][j]))
            
    DoM_b_all.append(np.mean(temp)-np.mean(non))

DoM_t_all = []
for i in t_all_row_indices:
    #print i 
    temp = []
    non = []
    for j in range(1,len(labels)):
        if labels[j] == 'T-ALL':
            temp.append(float(data[i][j]))
        else:
            non.append(float(data[i][j]))
            
    DoM_t_all.append(np.mean(temp)-np.mean(non))

DoM_aml = []
for i in aml_row_indices:
    #print i 
    temp = []
    non = []
    for j in range(1,len(labels)):
        if labels[j] == 'AML':
            temp.append(float(data[i][j]))
        else:
            non.append(float(data[i][j]))
            
    DoM_aml.append(np.mean(temp)-np.mean(non))

DoM_mds = []
for i in mds_row_indices:
    #print i 
    temp = []
    non = []
    for j in range(1,len(labels)):
        if labels[j] == 'leukemia class: MDS':
            temp.append(float(data[i][j]))
        else:
            non.append(float(data[i][j]))
            
    DoM_mds.append(np.mean(temp)-np.mean(non))
    
DoM_cml = []
for i in cml_row_indices:
    #print i 
    temp = []
    non = []
    for j in range(1,len(labels)):
        if labels[j] == 'leukemia class: CML':
            temp.append(float(data[i][j]))
        else:
            non.append(float(data[i][j]))
            
    DoM_cml.append(np.mean(temp)-np.mean(non))

DoM_cll = []
for i in cll_row_indices:
    #print i 
    temp = []
    non = []
    for j in range(1,len(labels)):
        if labels[j] == 'leukemia class: CLL':
            temp.append(float(data[i][j]))
        else:
            non.append(float(data[i][j]))
            
    DoM_cll.append(np.mean(temp)-np.mean(non))
    
    
DoM_healthy = []
for i in healthy_row_indices:
    #print i 
    temp = []
    non = []
    for j in range(1,len(labels)):
        if labels[j] == 'leukemia class: Non-leukemia and healthy bone marrow':
            temp.append(float(data[i][j]))
        else:
            non.append(float(data[i][j]))
            
    DoM_healthy.append(np.mean(temp)-np.mean(non))


# In[117]:

print (np.array(DoM_b_all).argsort()[:5])
print (len(DoM_b_all))


# In[122]:

# Take the 5 features with max and min DoM
subset = []
indices = []
k = 5

# min
for i in range(0, k):
    if b_all_row_indices[np.array(DoM_b_all).argsort()[:k][i]] not in indices:
        subset.append(data[b_all_row_indices[np.array(DoM_b_all).argsort()[:k][i]]][1:])
        indices.append(b_all_row_indices[np.array(DoM_b_all).argsort()[:k][i]])
    
#max
for i in range(0, k):
    if b_all_row_indices[np.array(DoM_b_all).argsort()[-k:][i]] not in indices:
        subset.append(data[b_all_row_indices[np.array(DoM_b_all).argsort()[-k:][i]]][1:])
        indices.append(b_all_row_indices[np.array(DoM_b_all).argsort()[-k:][i]])

#min
for i in range(0, k):
    if t_all_row_indices[np.array(DoM_t_all).argsort()[:k][i]] not in indices:
        subset.append(data[t_all_row_indices[np.array(DoM_t_all).argsort()[:k][i]]][1:])
        indices.append(t_all_row_indices[np.array(DoM_t_all).argsort()[:k][i]])
#max
for i in range(0, k):
    if t_all_row_indices[np.array(DoM_t_all).argsort()[-k:][i]] not in indices:
        subset.append(data[t_all_row_indices[np.array(DoM_t_all).argsort()[-k:][i]]][1:])
        indices.append(t_all_row_indices[np.array(DoM_t_all).argsort()[-k:][i]])
    
#min
for i in range(0, k):
    if aml_row_indices[np.array(DoM_aml).argsort()[:k][i]] not in indices:
        subset.append(data[aml_row_indices[np.array(DoM_aml).argsort()[:k][i]]][1:])
        indices.append(aml_row_indices[np.array(DoM_aml).argsort()[:k][i]])
#max
for i in range(0, k):
    if aml_row_indices[np.array(DoM_aml).argsort()[-k:][i]] not in indices:
        subset.append(data[aml_row_indices[np.array(DoM_aml).argsort()[-k:][i]]][1:])
        indices.append(aml_row_indices[np.array(DoM_aml).argsort()[-k:][i]])

#min
for i in range(0, k):
    if mds_row_indices[np.array(DoM_mds).argsort()[:k][i]] not in indices:
        subset.append(data[mds_row_indices[np.array(DoM_mds).argsort()[:k][i]]][1:])
        indices.append(mds_row_indices[np.array(DoM_mds).argsort()[:k][i]])
#max
for i in range(0, k):
    if mds_row_indices[np.array(DoM_mds).argsort()[-k:][i]] not in indices:
        subset.append(data[mds_row_indices[np.array(DoM_mds).argsort()[-k:][i]]][1:])
        indices.append(mds_row_indices[np.array(DoM_mds).argsort()[-k:][i]])

#min
for i in range(0, k):
    if cml_row_indices[np.array(DoM_cml).argsort()[:k][i]] not in indices:
        subset.append(data[cml_row_indices[np.array(DoM_cml).argsort()[:k][i]]][1:])
        indices.append(cml_row_indices[np.array(DoM_cml).argsort()[:k][i]])
#max
for i in range(0, k):
    if cml_row_indices[np.array(DoM_cml).argsort()[-k:][i]] not in indices:
        subset.append(data[cml_row_indices[np.array(DoM_cml).argsort()[-k:][i]]][1:])
        indices.append(cml_row_indices[np.array(DoM_cml).argsort()[-k:][i]])


#min
for i in range(0, k):
    if cll_row_indices[np.array(DoM_cll).argsort()[:k][i]] not in indices:
        subset.append(data[cll_row_indices[np.array(DoM_cll).argsort()[:k][i]]][1:])
        indices.append(cll_row_indices[np.array(DoM_cll).argsort()[:k][i]])
#max
for i in range(0, k):
    if cll_row_indices[np.array(DoM_cll).argsort()[-k:][i]] not in indices:
        subset.append(data[cll_row_indices[np.array(DoM_cll).argsort()[-k:][i]]][1:])
        indices.append(cll_row_indices[np.array(DoM_cll).argsort()[-k:][i]])


#min
for i in range(0, k):
    if healthy_row_indices[np.array(DoM_healthy).argsort()[:k][i]] not in indices:
        subset.append(data[healthy_row_indices[np.array(DoM_healthy).argsort()[:k][i]]][1:])
        indices.append(healthy_row_indices[np.array(DoM_healthy).argsort()[:k][i]])
#max
for i in range(0, k):
    if healthy_row_indices[np.array(DoM_healthy).argsort()[-k:][i]] not in indices:
        subset.append(data[healthy_row_indices[np.array(DoM_healthy).argsort()[-k:][i]]][1:])
        indices.append(healthy_row_indices[np.array(DoM_healthy).argsort()[-k:][i]])

print (len(subset))


# In[123]:

final_data = np.array(subset, dtype=float)


# In[124]:

print (final_data)


# In[125]:

np.savetxt("final595data.csv", final_data, delimiter=",")
index = []
for i in indices:
    index.append(data[i][0])
np.savetxt("final595indices.csv", np.array(index), delimiter=",", fmt='%s')

np.savetxt("final595labels.csv", np.array(labels), delimiter=",", fmt='%s')

np.savetxt("final595samplenames.csv", np.array(data[0]), delimiter=",", fmt='%s')


# In[ ]:



