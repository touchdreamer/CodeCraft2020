#include <bits/stdc++.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <fcntl.h>
using namespace std;
const int maxn = 2e6+10;
const long long maxm = 2e7;
#define PROC_COUNT 8
#define THREAD_COUNT 4
#define CIRCLE_NUM 5
#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)
typedef long long ll;

int ans3[maxm][3], ans4[maxm][4], ans5[maxm][5], ans6[maxm][6], ans7[maxm][7]; 
int circle_len[8];
char ans[maxm];

int total, len;

struct str{
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

struct Node{
     uint32_t from, to, val;
     bool operator < (const Node& temp)const{
          if(from != temp.from) return from > temp.from;
          else return to > temp.to;
     }
}node[maxn];

int indegree[maxn], outdegree[maxn], cnt, n;

struct FILE_IO{
     // input and output file name
     string input_file = "/data/test_data.txt";
     string output_file = "/projects/student/result.txt";
     //string output_file = "result.txt";

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
		while(*buf_begin != '\r'){
			x.val = (uint32_t)10 * x.val + (uint32_t)(*buf_begin - '0');
			buf_begin++;
		}
		node[cnt++].val = x.val;
		//buf_begin++;
	        buf_begin += 2;
	}
	munmap(buf, sz);
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
	 register int i;
	 for(i = 3; i <= 7; i++) total += circle_len[i];		
	 //cout << total << endl;
	 int sta[12], top = 0;
	 do{
		sta[++top] = total % 10;
		total /= 10;
	 }while(total);
	 char* mp_dst = start_point;
	 for(i = top; i; i--) *(mp_dst++) = 48 + sta[i];
	 *(mp_dst++) = '\n';
	 
	 for(i = circle_len[3] - 1; i >= 0; i--){
             memcpy(mp_dst, V[ans3[i][0]].c, V[ans3[i][0]].len);
	     mp_dst += V[ans3[i][0]].len;
	     *(mp_dst++) = ',';
	     memcpy(mp_dst, V[ans3[i][1]].c, V[ans3[i][1]].len);
	     mp_dst += V[ans3[i][1]].len;
	     *(mp_dst++) = ',';
	     memcpy(mp_dst, V[ans3[i][2]].c, V[ans3[i][2]].len);
	     mp_dst += V[ans3[i][2]].len;
	     *(mp_dst++) = '\n';
	 } 
	 for(i = circle_len[4] - 1; i >= 0; i--){
	     memcpy(mp_dst, V[ans4[i][0]].c, V[ans4[i][0]].len);
	     mp_dst += V[ans4[i][0]].len;
	     *(mp_dst++) = ',';
	     memcpy(mp_dst, V[ans4[i][1]].c, V[ans4[i][1]].len);
	     mp_dst += V[ans4[i][1]].len;
	     *(mp_dst++) = ',';
	     memcpy(mp_dst, V[ans4[i][2]].c, V[ans4[i][2]].len);
	     mp_dst += V[ans4[i][2]].len;
	     *(mp_dst++) = ',';
	     memcpy(mp_dst, V[ans4[i][3]].c, V[ans4[i][3]].len);
	     mp_dst += V[ans4[i][3]].len;
	     *(mp_dst++) = '\n';
	 } 
	 for(i = circle_len[5] - 1; i >= 0; i--){
	     memcpy(mp_dst, V[ans5[i][0]].c, V[ans5[i][0]].len);
	     mp_dst += V[ans5[i][0]].len;
	     *(mp_dst++) = ',';
	     memcpy(mp_dst, V[ans5[i][1]].c, V[ans5[i][1]].len);
	     mp_dst += V[ans5[i][1]].len;
	     *(mp_dst++) = ',';
	     memcpy(mp_dst, V[ans5[i][2]].c, V[ans5[i][2]].len);
	     mp_dst += V[ans5[i][2]].len;
	     *(mp_dst++) = ',';
	     memcpy(mp_dst, V[ans5[i][3]].c, V[ans5[i][3]].len);
	     mp_dst += V[ans5[i][3]].len;
	     *(mp_dst++) = ',';
	     memcpy(mp_dst, V[ans5[i][4]].c, V[ans5[i][4]].len);
	     mp_dst += V[ans5[i][4]].len;
	     *(mp_dst++) = '\n';
	 } 
	 for(i = circle_len[6] - 1; i >= 0; i--){
	     memcpy(mp_dst, V[ans6[i][0]].c, V[ans6[i][0]].len);
	     mp_dst += V[ans6[i][0]].len;
	     *(mp_dst++) = ',';
	     memcpy(mp_dst, V[ans6[i][1]].c, V[ans6[i][1]].len);
	     mp_dst += V[ans6[i][1]].len;
	     *(mp_dst++) = ',';
	     memcpy(mp_dst, V[ans6[i][2]].c, V[ans6[i][2]].len);
	     mp_dst += V[ans6[i][2]].len;
	     *(mp_dst++) = ',';
	     memcpy(mp_dst, V[ans6[i][3]].c, V[ans6[i][3]].len);
	     mp_dst += V[ans6[i][3]].len;
	     *(mp_dst++) = ',';
	     memcpy(mp_dst, V[ans6[i][4]].c, V[ans6[i][4]].len);
	     mp_dst += V[ans6[i][4]].len;
	     *(mp_dst++) = ',';
	     memcpy(mp_dst, V[ans6[i][5]].c, V[ans6[i][5]].len);
	     mp_dst += V[ans6[i][5]].len;
	     *(mp_dst++) = '\n';
	 } 
	 for(i = circle_len[7] - 1; i >= 0; i--){
	     memcpy(mp_dst, V[ans7[i][0]].c, V[ans7[i][0]].len);
	     mp_dst += V[ans7[i][0]].len;
	     *(mp_dst++) = ',';
	     memcpy(mp_dst, V[ans7[i][1]].c, V[ans7[i][1]].len);
	     mp_dst += V[ans7[i][1]].len;
	     *(mp_dst++) = ',';
	     memcpy(mp_dst, V[ans7[i][2]].c, V[ans7[i][2]].len);
	     mp_dst += V[ans7[i][2]].len;
	     *(mp_dst++) = ',';
	     memcpy(mp_dst, V[ans7[i][3]].c, V[ans7[i][3]].len);
	     mp_dst += V[ans7[i][3]].len;
	     *(mp_dst++) = ',';
	     memcpy(mp_dst, V[ans7[i][4]].c, V[ans7[i][4]].len);
	     mp_dst += V[ans7[i][4]].len;
	     *(mp_dst++) = ',';
	     memcpy(mp_dst, V[ans7[i][5]].c, V[ans7[i][5]].len);
	     mp_dst += V[ans7[i][5]].len;
	     *(mp_dst++) = ',';
	     memcpy(mp_dst, V[ans7[i][6]].c, V[ans7[i][6]].len);
	     mp_dst += V[ans7[i][6]].len;
	     *(mp_dst++) = '\n';
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

vector <Edge> e[maxn], re[maxn];

int Binary_search(int length, uint32_t& val){
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

bool vis[maxn];
vector <hidden_state> G[maxn];
int visited[maxn], Count, start;

//forward direction
void dfs(int u){
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
	       for(ii = 0; ii < G[v2].size(); ii++){  // backward edge
		     if(G[v2][ii].head_money > 5ll*e[u][i].val || e[u][i].val > 3ll*G[v2][ii].head_money) continue;
		     if(G[v2][ii].tail_money > 3ll*e[v1][j].val || e[v1][j].val > 5ll*G[v2][ii].tail_money) continue;
                     judge = true;
                     if(G[v2][ii].hidden_node.first == cir[1] || G[v2][ii].hidden_node.second == cir[1]) judge = false;
                     if(likely(judge)){ // the length of circle is five(5)
			 //total++;
			 cir[3] = G[v2][ii].hidden_node.first, cir[4] = G[v2][ii].hidden_node.second;
			 memcpy(ans5[circle_len[5]++], cir, 20);
                     }
               }
               for(k = 0; k < e[v2].size(); k++){
                    v3 = e[v2][k].to; // 3 + 3 = 6
		    if(e[v2][k].val > 3ll*e[v1][j].val || e[v1][j].val > 5ll*e[v2][k].val) continue;
		    if(v3 == u){ // The length of circle is three(3)
                         if(e[u][i].val > 3ll*e[v2][k].val || e[v2][k].val > 5ll*e[u][i].val) continue;
			 //total++;
			 memcpy(ans3[circle_len[3]++], cir, 12);
			 continue;
                    }
                    if(vis[v3]) continue;
                    vis[v3] = true;
		    cir[3] = v3;
	       	    for(ii = 0; ii < G[v3].size(); ii++){  // backward edge
		        if(G[v3][ii].head_money > 5ll*e[u][i].val || e[u][i].val > 3ll*G[v3][ii].head_money) continue;
		        if(G[v3][ii].tail_money > 3ll*e[v2][k].val || e[v2][k].val > 5ll*G[v3][ii].tail_money) continue;
			judge = true;
                        if(G[v3][ii].hidden_node.first == cir[1] || G[v3][ii].hidden_node.second == cir[1]) judge = false;
                        if(G[v3][ii].hidden_node.first == cir[2] || G[v3][ii].hidden_node.second == cir[2]) judge = false;

                        if(unlikely(judge)){ // The length of circle is six(6)
			    //total++;
			    cir[4] = G[v3][ii].hidden_node.first, cir[5] = G[v3][ii].hidden_node.second;
			    memcpy(ans6[circle_len[6]++], cir, 24);
                    	} 
		    }
		    for(d = 0; d < e[v3].size(); d++){
                         v = e[v3][d].to;  // 3 + 4 = 7
			 if(e[v3][d].val > 3ll*e[v2][k].val || e[v2][k].val > 5ll*e[v3][d].val) continue;
			 if(v == u){ // The length of circle is four(4)
                                if(e[u][i].val > 3ll*e[v3][d].val || e[v3][d].val > 5ll*e[u][i].val) continue;
				//total++;
				memcpy(ans4[circle_len[4]++], cir, 16);
				continue;
                         }
                         if(vis[v]) continue;
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
				   //total++;
			           cir[5] = G[v][ii].hidden_node.first, cir[6] = G[v][ii].hidden_node.second;
				   memcpy(ans7[circle_len[7]++], cir, 28);
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

void rdfs(int u){
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
		    if(!G[v3].size()) visited[Count++] = v3;
		    G[v3].push_back(hidden_state(make_pair(v2, v1), re[u][i].val, re[v2][k].val));
	       }
               vis[v2] = false;
          }
          vis[v1] = false;
     }
     vis[u] = false;
     for(i = 0; i < Count; i++)
     	sort(G[visited[i]].begin(), G[visited[i]].end());
}


void solve(){
     sort(node, node + cnt);
     sort(V, V + n);
     n = unique(V, V + n) - V;
     register int u, i = 0, j, from, to;
     from = Binary_search(n, node[i].from);
     to = Binary_search(n, node[i].to);
     for(u = n-1; u >= 0; u--){
	 while(from > u){
	 	from = Binary_search(n, node[i].from);
		to = Binary_search(n, node[i].to);
		i++;	
	 }
     	 while(i < cnt && from == u){
	 	e[from].push_back(Edge(to, node[i].val));
		re[to].push_back(Edge(from, node[i].val));
		indegree[to]++;
		outdegree[from]++;
		i++;
		if(i < cnt){
		    from = Binary_search(n, node[i].from);
		    to = Binary_search(n, node[i].to);
		}
	 }
 	 
	 if(indegree[u] && outdegree[u]){
	 	rdfs(u);
		dfs(u);
		for(j = 0; j < Count; j++) G[visited[j]].clear();
		Count = 0;
	 }
     }
}

int main(){
     //clock_t startTime, middleTime, endTime;
     //startTime = clock();
     io.read_from();
     //middleTime = clock();
     //cout << "input time:" << (double) (middleTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
     // find circle
     solve();
     //middleTime = clock();
     //cout << "Find Circle Time : " <<(double)(middleTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
     io.write_back();
     //endTime = clock();
     //cout << "Total Time : " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
     return 0;
}
