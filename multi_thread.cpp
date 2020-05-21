#include <bits/stdc++.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <thread>
#include <mutex>
using namespace std;
const int maxn = 2e6+10;
const long long maxm = 2e7;
#define PROC_COUNT 8
#define THREAD_COUNT 8
#define CIRCLE_NUM 5
#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)
typedef long long ll;

int ans0[(maxm>>2)*3], ans1[(maxm>>2)*4], ans2[(maxm>>1)*5], ans3[maxm*6], ans4[maxm*7], ans5[maxm*7], ans6[maxm*7], ans7[maxm*7]; 
int* ans[] = {ans0, ans1, ans2, ans3, ans4, ans5, ans6, ans7}; // store all answer

int circle_len[8], total;

struct str{ // store vertex
	int len;
	uint32_t val;
	char c[10];
	bool operator < (const str& temp){
		return val < temp.val;
	}

	bool operator == (const str& temp){
		return val == temp.val;
	}
}V[maxn << 1];

struct Node{ // store all edges
     uint32_t from, to, val;
     bool operator < (const Node& temp)const{
          if(from != temp.from) return from > temp.from;
          else return to > temp.to;
     }
}node[maxn];

int indegree0[maxn], indegree1[maxn], indegree2[maxn], indegree3[maxn], indegree4[maxn], indegree5[maxn], indegree6[maxn], indegree7[maxn];
int outdegree0[maxn], outdegree1[maxn], outdegree2[maxn], outdegree3[maxn], outdegree4[maxn], outdegree5[maxn], outdegree6[maxn], outdegree7[maxn];
int cnt, n;

struct FILE_IO{
     /* using mmap function to accelerate read and write processing
      * write_back() can use multi-processing accelerate write processing
      * */
     // input and output file name
     string input_file = "/data/test_data.txt";
     //string output_file = "/projects/student/result.txt";
     string output_file = "result.txt";

     void read_from(){
     	int fd_src = open(input_file.c_str(), O_RDONLY);
	if(fd_src < 0){
		perror("open file");
		exit(1);
	}
	int sz = lseek(fd_src, 0, SEEK_END);
	char* buf = (char*)mmap(NULL, sz, PROT_READ, MAP_PRIVATE, fd_src, 0);
	close(fd_src);
	char* buf_begin = buf;
	const char* buf_end = buf + sz;
	str x;
	while(buf_begin < buf_end){
	 	x.val = 0;
		x.len = 0;
		while(*buf_begin != ','){
			x.val = (uint32_t)10 * x.val + (uint32_t)(*buf_begin - '0');
			x.c[x.len++] = *buf_begin;
			buf_begin++;
		}
		buf_begin++;
		V[n++] = x;
                node[cnt].from = x.val;

	 	x.val = 0;
		x.len = 0;
		while(*buf_begin != ','){
			x.val = (uint32_t)10 * x.val + (uint32_t)(*buf_begin - '0');
			x.c[x.len++] = *buf_begin;
			buf_begin++;
		}
		buf_begin++;
		V[n++] = x;
                node[cnt].to = x.val;

	 	x.val = 0;
		while(*buf_begin != '\n'){
			x.val = (uint32_t)10 * x.val + (uint32_t)(*buf_begin - '0');
			buf_begin++;
		}
		node[cnt++].val = x.val;
		buf_begin++;
	        //buf_begin += 2;
	}
	munmap(buf, sz);
     }

     void multiprocess_write(){
	 int fd_dst = open(output_file.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0666);
	 if(fd_dst < 0){
		perror("open file error");
	 	exit(1);
	 }
	 register int i, j, k, len[8];
	 total = 0;
	 for(i = 0; i < THREAD_COUNT; i++) total += circle_len[i];
	 int sta[12], top = 0;
	 do{
		sta[++top] = total % 10;
		total /= 10;
	 }while(total);
	 int flen = 0;
	 for(i = 0; i < THREAD_COUNT; i++){
             len[i] = flen;
	     k = (i+3>7)?7:i+3;
	     for(j = 0; j < circle_len[i]*k; j++)
	     	 flen += V[ans[i][j]].len + 1; 
	 }
	 flen += top+1;
	 int ret = ftruncate(fd_dst, flen);
	 if(ret < 0){
	 	perror("ftruncate");
		exit(2);
	 }
	 char* mp_dst = (char*)mmap(NULL, flen, PROT_READ | PROT_WRITE, MAP_SHARED, fd_dst, 0);
	 if(mp_dst == MAP_FAILED){
	 	perror("mmap");
		exit(3);
	 }
	 close(fd_dst);
	 
	 for(i = top; i; i--) *(mp_dst++) = 48 + sta[i];
	 *(mp_dst++) = '\n';

         pid_t pid;
	 for(i = 0;i < PROC_COUNT; i++){
	     pid = fork();
	     if(pid <= 0) break;
	 }
         if(PROC_COUNT == i){ // host process
	     for(j = 0; j < PROC_COUNT; j++) wait(NULL);
	 }
	 else{ // branch process
	     int dis = i + 3;
	     if(dis > 7) dis = 7;
	     int ptr = 0;
             for(j = circle_len[i] - 1; j >= 0; j--){
	         for(k = j*dis; k < (j + 1)*dis; k++){
		     memcpy(mp_dst + len[i] + ptr, V[ans[i][k]].c, V[ans[i][k]].len);
	             ptr += V[ans[i][k]].len;
	             *(mp_dst + len[i] + ptr) = (k == (j+1)*dis-1)?'\n':',';
	             ptr++;	     
		 }    
	     }
	 }
	 munmap(mp_dst, flen);
     }

     void write_back(){
	 int fd_dst = open(output_file.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0666);
	 if(fd_dst < 0){
		perror("open file error");
	 	exit(1);
	 }
	 int flen = 1e9;
	 int ret = ftruncate(fd_dst, flen);
	 if(ret < 0){
	 	perror("ftruncate");
		exit(2);
	 }
	 char* start_point = (char*)mmap(NULL, flen, PROT_READ | PROT_WRITE, MAP_SHARED, fd_dst, 0);
	 if(start_point == MAP_FAILED){
	 	perror("mmap");
		exit(3);
	 }
	 register int i, j, k;
	 total = 0;
	 for(i = 0; i < THREAD_COUNT; i++) {
	     total += circle_len[i];
	     cout << circle_len[i] << endl;
	 }	     
	 cout << total << endl;
	 int sta[12], top = 0;
	 do{
		sta[++top] = total % 10;
		total /= 10;
	 }while(total);
	 char* mp_dst = start_point;
	 for(i = top; i; i--) *(mp_dst++) = 48 + sta[i];
	 *(mp_dst++) = '\n';
	 int dis = 3;
         for(j = 0; j < THREAD_COUNT; j++){
	    for(i = circle_len[j] - 1; i >= 0; i--){
	    	for(k = i * dis; k < (i + 1) * dis; k++){
		    memcpy(mp_dst, V[ans[j][k]].c, V[ans[j][k]].len);
	            mp_dst += V[ans[j][k]].len;
	            *(mp_dst++) = (k == (i+1)*dis-1)?'\n':',';	    
		}	
	    }
	    if(dis < 7) dis++;
	 }
	 ftruncate(fd_dst, mp_dst - start_point); 
	 munmap(start_point, mp_dst - start_point);
     	 close(fd_dst);
     }
}io;


struct Edge{
     int to;
     uint32_t val;
     Edge(){}
     Edge(int _to, uint32_t _val):to(_to), val(_val){}
};

// build forward graph and backward graph
vector <Edge> temp[maxn];
vector <Edge> e0[maxn], e1[maxn], e2[maxn], e3[maxn], e4[maxn], e5[maxn], e6[maxn], e7[maxn];
vector <Edge> re2[maxn], re3[maxn], re4[maxn], re5[maxn], re6[maxn], re7[maxn];

int Binary_search(int length, uint32_t& val){
     /* 
      *establish discretization into all input node
      * */
     int l = 0, r = length - 1;
     while(l <= r){
          int mid = (l + r) >> 1;
          if(V[mid].val == val) return mid;
          else if(V[mid].val > val) r = mid-1;
          else l = mid+1;
     }
     return -1;
}

struct hidden_state{
	/*
	 * store backward edge information
	 * */
	pair<int, int> hidden_node;
	uint32_t head_money, tail_money;
	hidden_state(){}
	hidden_state(pair<int, int> p, uint32_t h, uint32_t t):hidden_node(p), head_money(h), tail_money(t){}

	bool operator < (const hidden_state& hidden)const{
		if(hidden_node.first != hidden.hidden_node.first) 
			return hidden_node.first > hidden.hidden_node.first;
		else return hidden_node.second > hidden.hidden_node.second;
	}
};

bool vis0[maxn], vis1[maxn], vis2[maxn], vis3[maxn], vis4[maxn], vis5[maxn], vis6[maxn], vis7[maxn];
vector <hidden_state> G2[maxn], G3[maxn], G4[maxn], G5[maxn], G6[maxn], G7[maxn];
int visited2[maxn], visited3[maxn], visited4[maxn], visited5[maxn], visited6[maxn], visited7[maxn], Count[8];
int* visited[] = {visited2, visited3, visited4, visited5, visited6, visited7};

void dfs0(int u, vector<Edge> e[maxn], bool* vis){ 
    /* The function is to find circle length equals three(3) */
    register int i, j, k, cir[3];
    cir[0] = u;
    vis[u] = true;
    for(i = 0; i < e[u].size(); i++){
    	int& v1 = e[u][i].to;
	if(vis[v1]) continue;
	vis[v1] = true;
	cir[1] = v1;
	for(j = 0; j < e[v1].size(); j++){
	    int& v2 = e[v1][j].to;
	    if(vis[v2] || e[v1][j].val > 3ll*e[u][i].val || e[u][i].val > 5ll*e[v1][j].val) continue;
	    vis[v2] = true;
	    cir[2] = v2;
	    for(k = 0; k < e[v2].size(); k++){
	    	int& v3 = e[v2][k].to;
                if(v3 == u){
		    if(e[v2][k].val > 3ll*e[v1][j].val || e[v1][j].val > 5ll*e[v2][k].val) continue;
                    if(e[u][i].val > 3ll*e[v2][k].val || e[v2][k].val > 5ll*e[u][i].val) continue;
		    memcpy(ans[0] + circle_len[0] * 3, cir, 12);
		    circle_len[0]++;
		}
	    }
	    vis[v2] = false;
	}
	vis[v1] = false;
    }
    vis[u] = false;
}

void dfs1(int u, vector<Edge> e[maxn], bool* vis){ 
    /* The function is to find circle length equals four(4) */
    register int i, j, k, d, cir[4];
    cir[0] = u;
    vis[u] = true;
    for(i = 0; i < e[u].size(); i++){
    	int& v1 = e[u][i].to;
	if(vis[v1]) continue;
	vis[v1] = true;
	cir[1] = v1;
	for(j = 0; j < e[v1].size(); j++){
	    int& v2 = e[v1][j].to;
	    if(vis[v2] || e[v1][j].val > 3ll*e[u][i].val || e[u][i].val > 5ll*e[v1][j].val) continue;
	    vis[v2] = true;
	    cir[2] = v2;
	    for(k = 0; k < e[v2].size(); k++){
	    	int& v3 = e[v2][k].to;
		if(vis[v3] || e[v2][k].val > 3ll*e[v1][j].val || e[v1][j].val > 5ll*e[v2][k].val) continue;
                vis[v3] = true;
		cir[3] = v3;
		for(d = 0; d < e[v3].size(); d++){
		    int v = e[v3][d].to;
		    if(v == u){
			if(e[v3][d].val > 3ll*e[v2][k].val || e[v2][k].val > 5ll*e[v3][d].val) continue;
                    	if(e[u][i].val > 3ll*e[v3][d].val || e[v3][d].val > 5ll*e[u][i].val) continue;
		        memcpy(ans[1] + circle_len[1]*4, cir, 16); 
		        circle_len[1]++;	
	            }
		}
		vis[v3] = false;
	    }
	    vis[v2] = false;
	}
	vis[v1] = false;
    }
    vis[u] = false;
}

void dfs2(int u, vector <Edge> e[maxn], vector<hidden_state> G[maxn], bool* vis){
     /* This function is to find circle length equals five(5)*/
     register int i, j, v1, v2, ii, cir[5];
     register bool judge;
     cir[0] = u;
     vis[u] = true;
     for(i = 0; i < e[u].size(); i++){
          v1 = e[u][i].to;

	  if(vis[v1]) continue;
          vis[v1] = true;
          cir[1] = v1; 
	  for(j = 0; j < e[v1].size(); j++){
               v2 = e[v1][j].to; // 2 + 3 = 5
	       if(vis[v2] || e[v1][j].val > 3ll*e[u][i].val || e[u][i].val > 5ll*e[v1][j].val) continue;
	       vis[v2] = true;
               cir[2] = v2;
	       for(ii = 0; ii < G[v2].size(); ii++){  // backward edge
		     if(G[v2][ii].head_money > 5ll*e[u][i].val || e[u][i].val > 3ll*G[v2][ii].head_money) continue;
		     if(G[v2][ii].tail_money > 3ll*e[v1][j].val || e[v1][j].val > 5ll*G[v2][ii].tail_money) continue;
                     judge = true;
                     if(G[v2][ii].hidden_node.first == cir[1] || G[v2][ii].hidden_node.second == cir[1]) judge = false;
                     if(likely(judge)){ // the length of circle is five(5)
			 cir[3] = G[v2][ii].hidden_node.first, cir[4] = G[v2][ii].hidden_node.second;
		         memcpy(ans[2] + circle_len[2]*5, cir, 20); 
			 circle_len[2]++;
                     }
               }
               vis[v2] = false;
          }
          vis[v1] = false;
     }
     vis[u] = false;
}

void dfs3(int u, vector<Edge> e[maxn],  vector<hidden_state> G[maxn], bool* vis){
     /* This function is to find circle length equals six(6)*/
     register int i, j, k, d, v1, v2, v3, v, ii, cir[7];
     register bool judge;
     cir[0] = u;
     vis[u] = true;
     for(i = 0; i < e[u].size(); i++){
          v1 = e[u][i].to;

	  if(vis[v1]) continue;
          vis[v1] = true;
          cir[1] = v1; 
	  for(j = 0; j < e[v1].size(); j++){
               v2 = e[v1][j].to; // 2 + 3 = 5
	       if(vis[v2] || e[v1][j].val > 3ll*e[u][i].val || e[u][i].val > 5ll*e[v1][j].val) continue;
	       vis[v2] = true;
               cir[2] = v2;
               for(k = 0; k < e[v2].size(); k++){
                    v3 = e[v2][k].to; // 3 + 3 = 6
		    if(vis[v3] || e[v2][k].val > 3ll*e[v1][j].val || e[v1][j].val > 5ll*e[v2][k].val) continue;
                    vis[v3] = true;
		    cir[3] = v3;
	       	    for(ii = 0; ii < G[v3].size(); ii++){  // backward edge
		        if(G[v3][ii].head_money > 5ll*e[u][i].val || e[u][i].val > 3ll*G[v3][ii].head_money) continue;
		        if(G[v3][ii].tail_money > 3ll*e[v2][k].val || e[v2][k].val > 5ll*G[v3][ii].tail_money) continue;
			judge = true;
                        if(G[v3][ii].hidden_node.first == cir[1] || G[v3][ii].hidden_node.second == cir[1]) judge = false;
                        if(G[v3][ii].hidden_node.first == cir[2] || G[v3][ii].hidden_node.second == cir[2]) judge = false;

                        if(unlikely(judge)){ // The length of circle is six(6)
			    cir[4] = G[v3][ii].hidden_node.first, cir[5] = G[v3][ii].hidden_node.second;
		            memcpy(ans[3] + circle_len[3]*6, cir, 24);
			    circle_len[3]++;
                    	} 
		    }
		    vis[v3] = false;
	       }
	       vis[v2] = false;
	  }
	  vis[v1] = false;
     }
     vis[u] = false;
}

void dfs(int u, vector<Edge> e[maxn],  vector<hidden_state> G[maxn], bool* vis, int& tid){
     /* This function is to find circle length equals seven(7)*/
     register int i, j, k, d, v1, v2, v3, v, ii, cir[7];
     register bool judge;
     cir[0] = u;
     vis[u] = true;
     for(i = 0; i < e[u].size(); i++){
          v1 = e[u][i].to;

	  if(vis[v1]) continue;
          vis[v1] = true;
          cir[1] = v1; 
	  for(j = 0; j < e[v1].size(); j++){
               v2 = e[v1][j].to; // 2 + 3 = 5
	       if(vis[v2] || e[v1][j].val > 3ll*e[u][i].val || e[u][i].val > 5ll*e[v1][j].val) continue;
	       vis[v2] = true;
               cir[2] = v2;
               for(k = 0; k < e[v2].size(); k++){
                    v3 = e[v2][k].to; // 3 + 3 = 6
		    if(vis[v3] || e[v2][k].val > 3ll*e[v1][j].val || e[v1][j].val > 5ll*e[v2][k].val) continue;
                    vis[v3] = true;
		    cir[3] = v3;
		    for(d = 0; d < e[v3].size(); d++){
                         v = e[v3][d].to;  // 3 + 4 = 7
			 if(vis[v] || e[v3][d].val > 3ll*e[v2][k].val || e[v2][k].val > 5ll*e[v3][d].val) continue;
                         cir[4] = v;
			 vis[v] = true;
			 for(ii = 0; ii < G[v].size(); ii++){  // backward edge
		              if(G[v][ii].head_money > 5ll*e[u][i].val || e[u][i].val > 3ll*G[v][ii].head_money) continue;
		              if(G[v][ii].tail_money > 3ll*e[v3][d].val || e[v3][d].val > 5ll*G[v][ii].tail_money) continue;
			      judge = true;
                     	      if(G[v][ii].hidden_node.first == cir[1] || G[v][ii].hidden_node.second == cir[1]) judge = false;
                              if(G[v][ii].hidden_node.first == cir[2] || G[v][ii].hidden_node.second == cir[2]) judge = false;
                              if(G[v][ii].hidden_node.first == cir[3] || G[v][ii].hidden_node.second == cir[3]) judge = false;
			      
			      if(unlikely(judge)){ // The length of circle is seven(7)
			           cir[5] = G[v][ii].hidden_node.first, cir[6] = G[v][ii].hidden_node.second;
				   memcpy(ans[tid] + circle_len[tid] * 7, cir, 28);
                                   circle_len[tid]++;
                              }
                         }
			 vis[v] = false;
                    }
                    vis[v3] = false;
               }
               vis[v2] = false;
          }
          vis[v1] = false;
     }
     vis[u] = false;
}

void rdfs(int u, vector <Edge> re[maxn], vector<hidden_state> G[maxn], bool* vis, int& tid){
     vis[u] = true;
     register int i, j, k, v1, v2, v3;
     for(i = 0; i < re[u].size(); i++){
          v1 = re[u][i].to;
	  if(vis[v1]) continue;
          vis[v1] = true;

          for(j = 0; j < re[v1].size(); j++){
	       int v2 = re[v1][j].to;
	       if(vis[v2] || re[v1][j].val > 5ll*re[u][i].val || re[u][i].val > 3ll*re[v1][j].val) continue;
               vis[v2] = true;

       	       for(k = 0; k < re[v2].size(); k++){
                    v3 = re[v2][k].to;
		    if(vis[v3] || re[v2][k].val > 5ll*re[v1][j].val || re[v1][j].val > 3ll*re[v2][k].val) continue;
		    if(!G[v3].size()) visited[tid-2][Count[tid]++] = v3;
		    G[v3].push_back(hidden_state(make_pair(v2, v1), re[u][i].val, re[v2][k].val));
	       }
               vis[v2] = false;
          }
          vis[v1] = false;
     }
     vis[u] = false;
     for(i = 0; i < Count[tid]; i++)
     	sort(G[visited[tid-2][i]].begin(), G[visited[tid-2][i]].end());
}

void multi_solve(int tid){
    register int u, i;
    if(tid == 0){ // find circle length equals three(3)
    	for(u = n-1; u >= 0; u--){
	    for(i = 0; i < temp[u].size(); i++){
	    	e0[u].push_back(temp[u][i]);
		indegree0[temp[u][i].to]++;
		outdegree0[u]++;
	    }
	    if(indegree0[u] && outdegree0[u]) dfs0(u, e0, vis0);
	}
    }
    else if(tid == 1){ // find circle length equals four(4)
    	for(u = n-1; u >= 0; u--){
	    for(i = 0; i < temp[u].size(); i++){
	    	e1[u].push_back(temp[u][i]);
		indegree1[temp[u][i].to]++;
		outdegree1[u]++;
	    }
	    if(indegree1[u] && outdegree1[u]) dfs1(u, e1, vis1);
	}
    }
    else if(tid == 2){ // find circle length equals five(5)
        for(u = n-1; u >= 0; u--){
	    for(i = 0; i < temp[u].size(); i++){
	    	e2[u].push_back(temp[u][i]);
		re2[temp[u][i].to].push_back(Edge(u, temp[u][i].val));
		indegree2[temp[u][i].to]++;
		outdegree2[u]++;
	    }
	    if(indegree2[u] && outdegree2[u]){
		rdfs(u, re2, G2, vis2, tid);
		dfs2(u, e2, G2, vis2);
		for(i = 0; i < Count[tid]; i++) G2[visited[tid-2][i]].clear();
		Count[tid] = 0;
	    }
	}
    }
    else if(tid == 3){ // find circle length equals six(6)
        for(u = n-1; u >= 0; u--){
	    for(i = 0; i < temp[u].size(); i++){
	    	e3[u].push_back(temp[u][i]);
		re3[temp[u][i].to].push_back(Edge(u, temp[u][i].val));
		indegree3[temp[u][i].to]++;
		outdegree3[u]++;
	    }
	    if(indegree3[u] && outdegree3[u]){
		rdfs(u, re3, G3, vis3, tid);
		dfs3(u, e3, G3, vis3);
                for(i = 0; i < Count[tid]; i++) G3[visited[tid-2][i]].clear();
		Count[tid] = 0;
	    }
	}
    }
    else{ // find circle length equals seven(7)
	int len1 = n/20, len2 = 2*n/20, len3 = 4*n/20;
    	if(tid == 4){ // [0, n/20]
            for(u = n-1; u >= 0; u--){
		for(i = 0; i < temp[u].size(); i++){
	            e4[u].push_back(temp[u][i]);
		    re4[temp[u][i].to].push_back(Edge(u, temp[u][i].val));
		    indegree4[temp[u][i].to]++;
		    outdegree4[u]++;
	        }
		if(u > len1) continue;
	        if(indegree4[u] && outdegree4[u]){
	    	    rdfs(u, re4, G4, vis4, tid);
		    dfs(u, e4, G4, vis4, tid);
                    for(i = 0; i < Count[tid]; i++) G4[visited[tid-2][i]].clear();
		    Count[tid] = 0;
	        }
	    }
	}
	else if(tid == 5){ // (n/20, 2*n/20]
            for(u = n-1; u > len1; u--){
	        for(i = 0; i < temp[u].size(); i++){
	    	    e5[u].push_back(temp[u][i]);
		    re5[temp[u][i].to].push_back(Edge(u, temp[u][i].val));
		    indegree5[temp[u][i].to]++;
		    outdegree5[u]++;
	        }
		if(u > len2) continue;
	        if(indegree5[u] && outdegree5[u]){
	    	    rdfs(u, re5, G5, vis5, tid);
		    dfs(u, e5, G5, vis5, tid);
                    for(i = 0; i < Count[tid]; i++) G5[visited[tid-2][i]].clear();
		    Count[tid] = 0;
	        }
	    }
	}
	else if(tid == 6){ // (2*n/20 ~ 4*n/20]
            for(u = n-1; u > len2; u--){
	        for(i = 0; i < temp[u].size(); i++){
	    	    e6[u].push_back(temp[u][i]);
		    re6[temp[u][i].to].push_back(Edge(u, temp[u][i].val));
		    indegree6[temp[u][i].to]++;
		    outdegree6[u]++;
	        }
		if(u > len3) continue;
	        if(indegree6[u] && outdegree6[u]){
	    	    rdfs(u, re6, G6, vis6, tid);
		    dfs(u, e6, G6, vis6, tid);
                    for(i = 0; i < Count[tid]; i++) G6[visited[tid-2][i]].clear();
		    Count[tid] = 0;
	        }
	    }
	}
	else if(tid == 7){ // (4*n/20, n)
            for(u = n-1; u > len3; u--){
	        for(i = 0; i < temp[u].size(); i++){
	    	    e7[u].push_back(temp[u][i]);
		    re7[temp[u][i].to].push_back(Edge(u, temp[u][i].val));
		    indegree7[temp[u][i].to]++;
		    outdegree7[u]++;
	        }
	        if(indegree7[u] && outdegree7[u]){
	    	    rdfs(u, re7, G7, vis7, tid);
		    dfs(u, e7, G7, vis7, tid);
                    for(i = 0; i < Count[tid]; i++) G7[visited[tid-2][i]].clear();
		    Count[tid] = 0;
	        }
	    }
	}	
    }
}

void multi_threading(){
    sort(node, node + cnt);
    sort(V, V + n);
    n = unique(V, V + n) - V;
    for(int i = 0; i < cnt; i++){
    	int from = Binary_search(n, node[i].from);
	int to = Binary_search(n, node[i].to);
	temp[from].push_back(Edge(to, node[i].val));
    }
    thread tids[THREAD_COUNT];
    for(int tc = 0; tc < THREAD_COUNT; tc++) tids[tc] = thread(multi_solve, tc);
    for(int tc = 0; tc < THREAD_COUNT; tc++) tids[tc].join();
}

int main(){
     clock_t startTime, middleTime, endTime;
     startTime = clock();
     io.read_from();
     middleTime = clock();
     cout << "input time:" << (double) (middleTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
     // find circle
     multi_threading();
     middleTime = clock();
     cout << "Find Circle Time : " <<(double)(middleTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
     io.multiprocess_write();
     endTime = clock();
     cout << "Total Time : " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
     return 0;
}
