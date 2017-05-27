def readgraph():
    y = list(map(int,input().split()))
    N, M = y[0], y[1]
    r = 0
    G = {}
    E = dict()
    W = dict()
    V = list(range(N))
    for i in range(0,N):
        G[i] = set()
    lines = 0
    i = 0
    while lines < M:
        y = list(map(int,input().split()))
        if y[1] == r:
            lines = lines + 1
            continue
        G[y[0]] = G[y[0]] | {y[1]}
        E[i] = (y[0],y[1])
        W[(y[0],y[1])] = y[2]
        i = i + 1
        lines = lines + 1
    return (G, V, E, W, r)

def Arbo(G, V, E, W, r):
    final = {}
    G1 = {}
    A = {}
    P = {}
    for e in E.keys():
        final[e] = False
    for u in V:
        if u != r:
            minedge = (100,100)
            mincost = 10000
            for i in E.keys():
                a,b = E[i]
                if b == u:
                    if W[(a,b)] < mincost:
                        mincost = W[(a,b)]
                        minedge = (a,b)
            x,y = minedge
            P[y] = x
            if x in A.keys():
                A[x] = A[x] | {y}
            else:
                A[x] = {y}
    flag = False
    Cycles = []
    previous = {}
    marked = {}
    path = set()
    visited = set()
    global parent
    def visit(vertex):
        if vertex in visited:
            return False
        visited.add(vertex)
        path.add(vertex)
        for neighbour in A.get(vertex, ()):
            if neighbour in path or visit(neighbour):
                return True
        path.remove(vertex)
        return False
    cyc = any(visit(v) for v in A)
    if cyc == False:
        return A
    
    for p in V:
        previous[p] = p
        marked[p] = False
    for u in A.keys():
        if flag == True:
            break
        marked[u] = True
        start = 100
        def dfs(w):
            nonlocal Cycles, start, marked, previous, flag
            if flag == False:
                for v in A[w]:
                    if flag == True:
                        break
                    if v not in marked.keys():
                        continue
                    if not marked[v]:
                        marked[v] = True
                        previous[v] = w
                        if v in A.keys():
                            dfs(v)
                    else:
                        previous[v] = w
                        start = v
                        flag = True
                        break
        dfs(u)
        if flag == True:
            p = start
            while previous[p] != start:
                p = previous[p]
                Cycles.append(p)
            Cycles.append(start)
            break
    Cycles.reverse()
    Cedges = []
    if(len(Cycles) == 2):
        Cedges.append((Cycles[0],Cycles[-1]))
    else:
        for i in range(0,len(Cycles)-1):
            Cedges.append((Cycles[i],Cycles[i+1]))
        Cedges.append((Cycles[-1],Cycles[0]))
    node = max(Cycles)
    V_new = []
    E_new = {}
    W_new = {}
    G_new = {}
    for u in V:
        if u not in Cycles:
            V_new.append(u)
    V_new.append(node)
    for v in V_new:
        G_new[v] = set()
    for i in E.keys():
        a, b = E[i]
        if a not in Cycles and b in Cycles :
            G_new[a] = G_new[a] | {node}
            E_new[i] = (a,node)
            W_new[(a,node)] = W[(a,b)] - W[(P[b],b)]
        if a in Cycles and b not in Cycles :
            G_new[node] = G_new[node] | {b}
            E_new[i] = (node,b)
            W_new[(node,b)] = W[(a,b)]
        if a not in Cycles and b not in Cycles:
            G_new[a] = G_new[a] | {b}
            E_new[i] = (a,b)
            W_new[(a,b)] = W[(a,b)]
            
    A_new = Arbo(G_new, V_new, E_new, W_new, r)
    for u in A_new.keys():
        for v in A_new[u]:
            if v == node:
                e = (u,v)
                break
    for i in E_new.keys():
        if E_new[i] == e:
            edge = E[i]
    for i in range(len(Cedges)):
        if Cedges[i] == (P[E[i][1]],E[i][1]):
            break
    Cedges.pop(i)
    for e in Cedges:
        final[e] = True

    for u in A_new.keys():
        for v in A_new[u]:
            for i in E_new.keys():
                if E_new[i] == (u,v):
                    break
            final[E[i]] = True

    A1 = {}
    for e in final.keys():
        if final[e] == True:
            u,v = e
            A1[u] = A1.get(u,set()) | {v}
    return A1
            

    
G, V, E, W, r = readgraph()
Arb = Arbo(G, V, E, W, r)
cost = 0
edges = []
for u in Arb.keys():
    for v in Arb[u]:
        cost = cost + W[(u,v)]
        edges.append((u,v))
print(cost)
for e in edges:
    print(e[0],e[1])
    
