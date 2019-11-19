# CSCI 3302: Homework 3 -- Clustering and Classification
# Implementations of K-Means clustering and K-Nearest Neighbor classification
import pickle
import random
import copy
import pdb
import matplotlib.pyplot as plt
import numpy as np
import math
from hw3_data import *

# Lucas Laughlin
LAST_NAME = "Laughlin"


def visualize_data(data, cluster_centers_file):
  fig = plt.figure(1, figsize=(4,3))
  f = open(cluster_centers_file, 'rb')
  centers = pickle.load(f)
  f.close()

  km = KMeansClassifier()
  km._cluster_centers = centers

  colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']

  labels = []
  center_colors = []
  for pt in data:
    labels.append(colors[km.classify(pt) % len(colors)])

  for i in range(len(centers)):
    center_colors.append(colors[i])

  plt.scatter([d[0] for d in data], [d[1] for d in data], c=labels, marker='x')
  plt.scatter([c[0] for c in centers], [c[1] for c in centers], c=center_colors, marker='D')
  plt.title("K-Means Visualization")
  plt.show()


class KMeansClassifier(object):

  def __init__(self):
    self._cluster_centers = [] # List of cluster centers, each of which is a point. ex: [ [10,10], [2,1], [0,-3] ]
    self._data = [] # List of datapoints (list of immutable lists, ex:  [ (0,0), (1.,5.), (2., 3.) ] )

  def add_datapoint(self, datapoint):
    self._data.append(datapoint)

  def fit(self, k):
    # Fit k clusters to the data, by starting with k randomly selected cluster centers.
    self._cluster_centers = [] # Reset cluster centers array

    self._cluster_centers = [self._data[i] for i in random.sample(xrange(0,len(self._data)), k)]

    while True:
      clusters=[]
      clusters = list(self._cluster_centers)
      for i in xrange(len(clusters)):
        clusters[i] = [clusters[i]]
      print(clusters)
      # Iterate through each datapoint in self._data and figure out which cluster it belongs to
      for p in self._data:
        clusters[self.classify(p)].append(p)
      
      
      print(clusters)
      print("\n")
      # Figure out new positions for each cluster center (should be the average position of all its points)
      new_cluster_centers =[np.mean(c, axis=0) for c in clusters]
      print(new_cluster_centers)
      # Check to see how much the cluster centers have moved (for the stopping condition)
      point_shift_sum = 0
      for i in range(len(self._cluster_centers)):   
        point_shift_sum += math.sqrt(sum([(a-b)**2 for a, b in zip(self._cluster_centers[i], new_cluster_centers[i])]))

      self._cluster_centers=list(new_cluster_centers)
      if point_shift_sum<1: # TODO: If the centers have moved less than some predefined threshold (you choose!) then exit the loop
        break      
    # TODO Add each of the 'k' final cluster_centers to the model (self._cluster_centers)

  def classify(self,p):
    # Given a data point p, figure out which cluster it belongs to and return that cluster's ID (its index in self._cluster_centers)
    #print(p)
    dist=[]
    for c in self._cluster_centers:
      #print(c)
      dist.append(math.sqrt(sum([(a-b)**2 for a, b in zip(c, p)])))
    #print(dist)
    closest_cluster_index = dist.index(min(dist))
    #print(closest_cluster_index)
    return closest_cluster_index

class KNNClassifier(object):

  def __init__(self):
    self._data = [] # list of (datapoint, label) tuples
  
  def clear_data(self):
    # Removes all data stored within the model
    self._data = []

  def add_labeled_datapoint(self, data_point, label):
    # Adds a labeled datapoint tuple onto the object's _data member
    self._data.append((data_point, label))
  
  def classify_datapoint(self, data_point, k):
    label_counts = {} # Dictionary mapping "label" => vote count
    best_label = None

    # Perform k_nearest_neighbor classification, setting best_label to the majority-vote label for k-nearest points
    #TODO: Find the k nearest points in self._data to data_point
    #TODO: Populate label_counts with the number of votes each label got from the k nearest points
    #TODO: Make sure to scale the weight of the vote each point gets by how far away it is from data_point
    #      Since you're just taking the max at the end of the algorithm, these do not need to be normalized in any way

    
    return best_label



def print_and_save_cluster_centers(classifier, filename):
  for idx, center in enumerate(classifier._cluster_centers):
    print("  Cluster %d, center at: %s" % (idx, str(center)))


  f = open(filename,'wb')
  pickle.dump(classifier._cluster_centers, f)
  f.close()

def read_data_file(filename):
  f = open(filename)
  data_dict = pickle.load(f)
  f.close()

  return data_dict['data'], data_dict['labels']

def read_hw_data():
  global hw_data
  data_dict = pickle.loads(hw_data)
  return data_dict['data'], data_dict['labels']

def main():
  global LAST_NAME
  # read data file
  #data, labels = read_data_file('hw3_data.pkl')

  # load dataset
  data, labels = read_hw_data()

  # data is an 'N' x 'M' matrix, where N=number of examples and M=number of dimensions per example
  # data[0] retrieves the 0th example, a list with 'M' elements, one for each dimension (xy-points would have M=2)
  # labels is an 'N'-element list, where labels[0] is the label for the datapoint at data[0]


  ########## PART 1 ############
  # perform K-means clustering
  kMeans_classifier = KMeansClassifier()
  for datapoint in data:
    kMeans_classifier.add_datapoint(datapoint) # add data to the model

  kMeans_classifier.fit(4) # Fit 4 clusters to the data

  # plot results
  print('\n'*2)
  print("K-means Classifier Test")
  print('-'*40)
  print("Cluster center locations:")
  print_and_save_cluster_centers(kMeans_classifier, "hw3_kmeans_" + LAST_NAME + ".pkl")

  print('\n'*2)


  ########## PART 2 ############
  print("K-Nearest Neighbor Classifier Test")
  print('-'*40)

  # Create and test K-nearest neighbor classifier
  kNN_classifier = KNNClassifier()
  k = 2

  correct_classifications = 0
  # Perform leave-one-out cross validation (LOOCV) to evaluate KNN performance
  for holdout_idx in range(len(data)):
    # Reset classifier
    kNN_classifier.clear_data()

    for idx in range(len(data)):
      if idx == holdout_idx: continue # Skip held-out data point being classified

      # Add (data point, label) tuples to KNNClassifier
      kNN_classifier.add_labeled_datapoint(data[idx], labels[idx])

    guess = kNN_classifier.classify_datapoint(data[holdout_idx], k) # Perform kNN classification
    if guess == labels[holdout_idx]: 
      correct_classifications += 1.0
  
  print("kNN classifier for k=%d" % k)
  print("Accuracy: %g" % (correct_classifications / len(data)))
  print('\n'*2)

  visualize_data(data, 'hw3_kmeans_' + LAST_NAME + '.pkl')


if __name__ == '__main__':
  main()
