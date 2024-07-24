import numpy as np

def pairwise_dist(x, y):
    """
    Args:
        x: N x D numpy array
        y: M x D numpy array
    Return:
            dist: N x M array, where dist2[i, j] is the euclidean distance between
            x[i, :] and y[j, :]
    """
    return np.sqrt(np.maximum(np.sum(x**2, axis=1).reshape(-1, 1) + np.sum(y**2, axis=1).reshape(1,-1) - 2*np.matmul(x,y.T),0))

class DBSCAN(object):
    def __init__(self, eps, minPts, dataset):
        self.eps = eps
        self.minPts = minPts
        self.dataset = dataset

    def fit(self):
        """
        Fits DBSCAN to dataset and hyperparameters defined in init().
        Args:
            None
        Return:
            cluster_idx: (N, ) int numpy array of assignment of clusters for each point in dataset
        Hint: Using sets for visitedIndices may be helpful here.
        Iterate through the dataset sequentially and keep track of your points' cluster assignments.
        See in what order the clusters are being expanded and new points are being checked, recommended to check neighbors of a point then decide whether to expand the cluster or not.
        If a point is unvisited or is a noise point (has fewer than the minimum number of neighbor points), then its cluster assignment should be -1.
        Set the first cluster as C = 0
        """
        cluster_idx = np.full(len(self.dataset), -1) 
        visitedIndices = set()
        C = 0
        
        for index in range(len(self.dataset)):
            if index not in visitedIndices:
                visitedIndices.add(index)
                neighborIndices = self.regionQuery(index)
                if len(neighborIndices) < self.minPts:
                    cluster_idx[index] = -1
                else:
                    self.expandCluster(index, neighborIndices, C, cluster_idx, visitedIndices)
                    C += 1
                    
        return cluster_idx

    def expandCluster(self, index, neighborIndices, C, cluster_idx, visitedIndices):
        """
        Expands cluster C using the point P, its neighbors, and any points density-reachable to P and updates indices visited, cluster assignments accordingly
           HINT: regionQuery could be used in your implementation
        Args:
            index: index of point P in dataset (self.dataset)
            neighborIndices: (N, ) int numpy array, indices of all points witin P's eps-neighborhood
            C: current cluster as an int
            cluster_idx: (N, ) int numpy array of current assignment of clusters for each point in dataset
            visitedIndices: set of indices in dataset visited so far
        Return:
            None
        """
        cluster_idx[index] = C
        i = 0
        while i < len(neighborIndices):
            neighbor = neighborIndices[i]
            if neighbor not in visitedIndices:
                visitedIndices.add(neighbor)
                neighbor_neighbors = self.regionQuery(neighbor)
                if len(neighbor_neighbors) >= self.minPts:
                    neighborIndices = np.concatenate((neighborIndices, np.setdiff1d(neighbor_neighbors, neighborIndices, assume_unique=True)))
            if cluster_idx[neighbor] == -1:
                cluster_idx[neighbor] = C
            i += 1

    def regionQuery(self, pointIndex):
        """
        Returns all points within P's eps-neighborhood (including P)

        Args:
            pointIndex: index of point P in dataset (self.dataset)
        Return:
            indices: (I, ) int numpy array containing the indices of all points within P's eps-neighborhood
        """
        return np.argwhere(pairwise_dist(self.dataset[pointIndex].reshape(1, -1), self.dataset)[0] <= self.eps).flatten()
