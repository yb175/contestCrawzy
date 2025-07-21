# Why Comp function is written Outside the class ?

**If We write inside the class we have to use a extra this function because it will be used as member functions**
---

---

```C++
bool comp(pair<int,int> a , pair<int,int> b){
        return a.second>b.second ; 
}
class Solution {
public:
    vector<pair<int,int>> pathCosts ; 
    void dfs(int src,int dest,vector<bool> &seen,int cost,vector<vector<pair<int,int>>>&adj,vector<bool> &online,int minpath){
        if(src==dest){
            pathCosts.push_back({cost,minpath});
        }
        seen[src]=1 ;
        for(auto neigh : adj[src]){
            if(!seen[neigh.first] && online[neigh.first]){
                seen[neigh.first]=1 ; 
                minpath = min(minpath,neigh.second) ; 
                dfs(neigh.first,dest,seen,cost+neigh.second,adj,online,minpath);
                seen[neigh.first]= 0 ; 
            }
        }
    }
    int findMaxPathScore(vector<vector<int>>& edges, vector<bool>& online, long long k) {
        int n = online.size();
        vector<vector<pair<int,int>>> adj(n);
        for(int i=0 ;i<edges.size();i++){
            int u = edges[i][0];
            int v = edges[i][1];
            int cost = edges[i][2];
            adj[u].push_back({v,cost});
        }

        vector<bool> seen(n,0);
        seen[0]=1 ; 
        dfs(0,n-1,seen,0,adj,online,INT_MAX);
        sort(pathCosts.begin(),pathCosts.end(),comp); 
        for(int i=0 ;i<pathCosts.size();i++){
            if(pathCosts[i].first<=k) return pathCosts[i].second ; 
        }
        return -1 ;
    }
};
```

# Why Above code Failed For graph ?

![img](https://i.ibb.co/3yk3g7Lb/Screenshot-2025-07-20-203407.png)

**Due to line**
```C++
 minpath = min(minpath,neigh.second) ; 
```
---
**This line was not valid for all the paths, because min path was already updated as it was directly passed because There would be parmanent changes in minPath**
---

# Brute Force Approach
```Cpp
bool comp(pair<int,int> a , pair<int,int> b){
        return a.second>b.second ; 
}
class Solution {
public:
    vector<pair<int,int>> pathCosts ; 
    void dfs(int src,int dest,vector<bool> &seen,int cost,vector<vector<pair<int,int>>>&adj,vector<bool> &online,int minpath){
        if(src==dest){
            pathCosts.push_back({cost,minpath});
            seen[src]=0 ;
            return ; 
        }
        seen[src]=1 ;
        for(auto neigh : adj[src]){
            if(!seen[neigh.first] && online[neigh.first]){
                seen[neigh.first]=1 ; 
                int newMin = min(minpath, neigh.second);
                dfs(neigh.first,dest,seen,cost+neigh.second,adj,online,newMin);
                seen[neigh.first]= 0 ; 
            }
        }
    }
    int findMaxPathScore(vector<vector<int>>& edges, vector<bool>& online, long long k) {
        int n = online.size();
        vector<vector<pair<int,int>>> adj(n);
        for(int i=0 ;i<edges.size();i++){
            int u = edges[i][0];
            int v = edges[i][1];
            int cost = edges[i][2];
            adj[u].push_back({v,cost});
        }

        vector<bool> seen(n,0);
        dfs(0,n-1,seen,0,adj,online,INT_MAX);
        sort(pathCosts.begin(),pathCosts.end(),comp); 
        for(int i=0 ;i<pathCosts.size();i++){
            if(pathCosts[i].first<=k) return pathCosts[i].second ; 
        }
        return -1 ;
    }
};
```
# Time Complexity 

O(Total paths*log(Total paths) 

---
# Thinking to Optimise above Approach 

---
## Thougth process for optimised Approach

**- Here we would use binary search and dijkstra Algorithm**
**-We would select a minEdge length from the range 0 to maxEdge Length**
**-We would apply dijkstra algorithm by filtering the path in which all the nodes are online and cost is greater than minEdge if we found any such path we can return true**
**-If result from diskstra is true then we can move to right**
**-Else we would decrease the search area**

# Code Part 
```C++
// Trying DijKstra Algorithm 
    vector<vector<pair<int,int>>> adj ; 
    bool dijkstra(int minEdge, vector<bool>& online , long long k){
        int n = online.size() ; 
        // For keeping track of elements marked 
        vector<bool> marked(n,0) ; 
        // For keeping track of dist
        vector<long long> dist(n,LLONG_MAX); 
        priority_queue<pair<long long, int>, vector<pair<long long, int>>, greater<pair<long long, int>>> pq ; 
        // Initialisation 
        dist[0]=0 ; 
        pq.push({0,0}) ; 

        for(int i= 0 ;i<n ;i++){
            if(dist[n-1]!=LLONG_MAX) return true ;
            while(!pq.empty() && marked[pq.top().second]) pq.pop(); 
            if (pq.empty()) break; 
            // unseen node found 
            int node = pq.top().second;
            pq.pop(); 

            // Marking The node seen 
            marked[node]=true ; 

            // Relaxation of edges 
            for(auto neighPair : adj[node]){
                int val = neighPair.first ;
                long long weight = neighPair.second ; 
                if(!marked[val] && online[val] && weight>=minEdge && weight+dist[node]<=k){
                    dist[val]=min(dist[val],weight+dist[node]) ; 
                    pq.push({dist[val],val}) ; 
                }
            }
        }

        return dist[n-1]!=LLONG_MAX ;
    }

    int findMaxPathScore(vector<vector<int>>& edges, vector<bool>& online, long long k) {
        int n = online.size(); // Number of nodes 
        adj.resize(n) ; 
        int maxEdge = 0 ; // Edge with maximum weight 

        // Built an adjcancy list 
        for(int i=0 ; i<edges.size();i++){
            int u = edges[i][0]; 
            int v = edges[i][1]; 
            int w = edges[i][2]; 
            maxEdge = max(maxEdge,w) ; 
            adj[u].push_back({v,w}); 
        }
        
        // Binary Search 
        int start = 0 ;
        int end = maxEdge ; 
        int ans = -1 ; 
        while(start<=end){
            int mid = start+(end-start)/2 ; 

            // Possible Answer , so checking greater value 
            if(dijkstra(mid,online,k)){
                ans = mid ; 
                start = mid+1 ; 
            }

            // Not answer found so trying smaller value 
            else end = mid-1 ; 
        }
        return ans ; 
    }
```

# Some Test Cases
![tcase1](https://i.ibb.co/gFgjsGMV/Screenshot-2025-07-21-142743.png)
![tcase2](https://i.ibb.co/Y4yPfx54/Screenshot-2025-07-21-143030.png)

# Time And Space Complexity 
## Time Complexity : O(log(maxEdgeLen)*(n^2 * log(n))

## Space complexity : O(log(maxEdgeLen)*(n^2))

# Problem Link : [network-recovery-pathways](https://leetcode.com/problems/network-recovery-pathways/)
