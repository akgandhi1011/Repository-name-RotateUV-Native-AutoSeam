#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

struct Vec3 { double x=0, y=0, z=0; };
struct Face { std::array<int,3> v{{0,0,0}}; };
struct ObjCorner { int v=0; int vt=0; };
struct ObjFace { std::array<ObjCorner,3> c{}; };
struct ObjMesh {
    std::vector<Vec3> vertices; // 1-based externally, 0-based storage
    std::vector<std::array<double,2>> tex;
    std::vector<ObjFace> faces;
};

struct EdgeKey {
    int a=0,b=0;
    EdgeKey() = default;
    EdgeKey(int x,int y) { if(x<y){a=x;b=y;} else {a=y;b=x;} }
    bool operator==(const EdgeKey& o) const { return a==o.a && b==o.b; }
    bool operator<(const EdgeKey& o) const { return a<o.a || (a==o.a && b<o.b); }
};
struct EdgeHash { size_t operator()(const EdgeKey& e) const noexcept { return (static_cast<size_t>(e.a)<<32) ^ static_cast<unsigned>(e.b); } };

static std::string quote(const fs::path& p) {
    std::string s = p.string();
    std::string out = "\"";
    for(char ch: s) { if(ch=='\"') out += "\\\""; else out += ch; }
    out += "\"";
    return out;
}

static bool parseIndexToken(const std::string& tok, int& v, int& vt) {
    v = vt = 0;
    if(tok.empty()) return false;
    auto p1 = tok.find('/');
    try {
        if(p1 == std::string::npos) { v = std::stoi(tok); return true; }
        v = std::stoi(tok.substr(0,p1));
        auto p2 = tok.find('/', p1+1);
        std::string t = tok.substr(p1+1, (p2==std::string::npos?tok.size():p2) - (p1+1));
        if(!t.empty()) vt = std::stoi(t);
        return true;
    } catch(...) { return false; }
}

static bool readTriObj(const fs::path& path, ObjMesh& mesh, std::string& err) {
    std::ifstream in(path);
    if(!in) { err = "Cannot open OBJ: " + path.string(); return false; }
    std::string line;
    while(std::getline(in,line)) {
        if(line.size() < 2) continue;
        std::istringstream ss(line);
        std::string tag; ss >> tag;
        if(tag=="v") {
            Vec3 p; if(!(ss>>p.x>>p.y>>p.z)) { err="Malformed vertex in OBJ."; return false; }
            mesh.vertices.push_back(p);
        } else if(tag=="vt") {
            double u=0,v=0; if(!(ss>>u>>v)) { err="Malformed vt in OBJ."; return false; }
            mesh.tex.push_back({u,v});
        } else if(tag=="f") {
            std::vector<ObjCorner> corners;
            std::string tok;
            while(ss>>tok) {
                int vi=0,vti=0;
                if(!parseIndexToken(tok,vi,vti)) { err="Malformed face token in OBJ."; return false; }
                if(vi < 0) vi = static_cast<int>(mesh.vertices.size()) + vi + 1;
                if(vti < 0) vti = static_cast<int>(mesh.tex.size()) + vti + 1;
                corners.push_back({vi,vti});
            }
            if(corners.size()!=3) { err="RotateUV Auto Seam worker requires triangulated OBJ input."; return false; }
            ObjFace f; for(int i=0;i<3;i++) f.c[i]=corners[i]; mesh.faces.push_back(f);
        }
    }
    if(mesh.vertices.empty() || mesh.faces.empty()) { err="OBJ has no vertices/faces."; return false; }
    return true;
}

static std::vector<std::vector<int>> faceComponents(const ObjMesh& m) {
    std::unordered_map<EdgeKey,std::vector<int>,EdgeHash> edgeFaces;
    for(int fi=0; fi<(int)m.faces.size(); ++fi) {
        const auto& f=m.faces[fi];
        for(int k=0;k<3;k++) edgeFaces[EdgeKey(f.c[k].v, f.c[(k+1)%3].v)].push_back(fi);
    }
    std::vector<std::vector<int>> adj(m.faces.size());
    for(auto& kv: edgeFaces) {
        const auto& fs=kv.second;
        for(size_t i=0;i<fs.size();++i) for(size_t j=i+1;j<fs.size();++j) {
            adj[fs[i]].push_back(fs[j]); adj[fs[j]].push_back(fs[i]);
        }
    }
    std::vector<char> seen(m.faces.size(),0);
    std::vector<std::vector<int>> comps;
    for(int s=0;s<(int)m.faces.size();++s) if(!seen[s]) {
        std::queue<int> q; q.push(s); seen[s]=1; std::vector<int> c;
        while(!q.empty()) { int f=q.front();q.pop(); c.push_back(f); for(int n:adj[f]) if(!seen[n]){seen[n]=1;q.push(n);} }
        comps.push_back(std::move(c));
    }
    return comps;
}

struct ComponentExport {
    fs::path path;
    std::vector<int> localToGlobal; // local 1-based index => global original 1-based; [0] unused
};

static bool writeComponentObj(const ObjMesh& input, const std::vector<int>& faceIds, const fs::path& path, ComponentExport& exp, std::string& err) {
    std::set<int> used;
    for(int fi:faceIds) for(auto& c:input.faces[fi].c) used.insert(c.v);
    std::unordered_map<int,int> globalToLocal;
    exp.localToGlobal.assign(1,0);
    int idx=1;
    for(int gv:used) { globalToLocal[gv]=idx++; exp.localToGlobal.push_back(gv); }
    std::ofstream out(path);
    if(!out) { err="Cannot create component OBJ."; return false; }
    out<<std::setprecision(17);
    for(int gv:used) { const auto&p=input.vertices[gv-1]; out<<"v "<<p.x<<" "<<p.y<<" "<<p.z<<"\n"; }
    for(int fi:faceIds) {
        auto& f=input.faces[fi];
        out<<"f "<<globalToLocal[f.c[0].v]<<" "<<globalToLocal[f.c[1].v]<<" "<<globalToLocal[f.c[2].v]<<"\n";
    }
    exp.path=path;
    return true;
}

struct QuantKey { long long x=0,y=0,z=0; bool operator==(const QuantKey&o)const{return x==o.x&&y==o.y&&z==o.z;} };
struct QuantHash { size_t operator()(const QuantKey&k)const noexcept { size_t h=std::hash<long long>{}(k.x); h^=std::hash<long long>{}(k.y)+0x9e3779b9+(h<<6)+(h>>2); h^=std::hash<long long>{}(k.z)+0x9e3779b9+(h<<6)+(h>>2); return h; } };
static double sqDist(const Vec3&a,const Vec3&b){double x=a.x-b.x,y=a.y-b.y,z=a.z-b.z;return x*x+y*y+z*z;}

static bool mapOutputVertices(const ObjMesh& input, const ObjMesh& out, std::vector<int>& outToInput, std::string& err) {
    Vec3 mn=input.vertices[0], mx=input.vertices[0];
    for(auto&p:input.vertices){mn.x=std::min(mn.x,p.x);mn.y=std::min(mn.y,p.y);mn.z=std::min(mn.z,p.z);mx.x=std::max(mx.x,p.x);mx.y=std::max(mx.y,p.y);mx.z=std::max(mx.z,p.z);}
    double diag=std::sqrt(sqDist(mn,mx));
    double tol=std::max(1e-9,diag*1e-7);
    double inv=1.0/tol;
    std::unordered_map<QuantKey,std::vector<int>,QuantHash> buckets;
    for(int i=0;i<(int)input.vertices.size();++i){auto&p=input.vertices[i];QuantKey k{llround(p.x*inv),llround(p.y*inv),llround(p.z*inv)};buckets[k].push_back(i+1);}
    outToInput.assign(out.vertices.size()+1,0);
    double maxSq=tol*tol*9.0;
    for(int oi=1;oi<=(int)out.vertices.size();++oi){
        auto&p=out.vertices[oi-1]; QuantKey base{llround(p.x*inv),llround(p.y*inv),llround(p.z*inv)};
        int best=0; double bestD=std::numeric_limits<double>::infinity();
        for(int dx=-1;dx<=1;dx++)for(int dy=-1;dy<=1;dy++)for(int dz=-1;dz<=1;dz++){
            QuantKey k{base.x+dx,base.y+dy,base.z+dz}; auto it=buckets.find(k); if(it==buckets.end())continue;
            for(int ii:it->second){double d=sqDist(p,input.vertices[ii-1]); if(d<bestD){bestD=d;best=ii;}}
        }
        if(best==0 || bestD>maxSq){err="Could not map an OptCuts output vertex back to the input mesh.";return false;}
        outToInput[oi]=best;
    }
    return true;
}

static std::string triKey(int a,int b,int c){std::array<int,3>x{{a,b,c}};std::sort(x.begin(),x.end());return std::to_string(x[0])+","+std::to_string(x[1])+","+std::to_string(x[2]);}

static bool deriveSeams(const ObjMesh& input, const ObjMesh& out, std::set<EdgeKey>& seams, int& unmatchedFaces, std::string& err) {
    std::vector<int> outToInput;
    if(!mapOutputVertices(input,out,outToInput,err)) return false;
    std::unordered_map<std::string,std::vector<int>> inFaceByKey;
    for(int fi=0;fi<(int)input.faces.size();++fi){auto&f=input.faces[fi];inFaceByKey[triKey(f.c[0].v,f.c[1].v,f.c[2].v)].push_back(fi);}
    std::unordered_map<std::string,int> usedCount;
    struct Corr { std::array<int,3> outV{{0,0,0}}; std::array<int,3> outVT{{0,0,0}}; bool ok=false; };
    std::vector<Corr> corr(input.faces.size()); unmatchedFaces=0;
    for(const auto& of:out.faces){
        int m0=(of.c[0].v>0&&of.c[0].v<(int)outToInput.size())?outToInput[of.c[0].v]:0;
        int m1=(of.c[1].v>0&&of.c[1].v<(int)outToInput.size())?outToInput[of.c[1].v]:0;
        int m2=(of.c[2].v>0&&of.c[2].v<(int)outToInput.size())?outToInput[of.c[2].v]:0;
        if(!m0||!m1||!m2){unmatchedFaces++;continue;}
        std::string key=triKey(m0,m1,m2); auto it=inFaceByKey.find(key); if(it==inFaceByKey.end()){unmatchedFaces++;continue;}
        int n=usedCount[key]++; if(n>=(int)it->second.size()){unmatchedFaces++;continue;} int fi=it->second[n];
        auto& inf=input.faces[fi]; Corr cc; cc.ok=true;
        for(int k=0;k<3;k++){
            int gv=inf.c[k].v; bool found=false;
            for(int j=0;j<3;j++) if(outToInput[of.c[j].v]==gv){cc.outV[k]=of.c[j].v;cc.outVT[k]=of.c[j].vt;found=true;break;}
            if(!found){cc.ok=false;break;}
        }
        corr[fi]=cc;
    }
    std::unordered_map<EdgeKey,std::vector<int>,EdgeHash> edgeFaces;
    for(int fi=0;fi<(int)input.faces.size();++fi){auto&f=input.faces[fi];for(int k=0;k<3;k++)edgeFaces[EdgeKey(f.c[k].v,f.c[(k+1)%3].v)].push_back(fi);}
    auto endpointData=[&](int fi,int gv)->std::pair<int,int>{
        auto&f=input.faces[fi]; for(int k=0;k<3;k++)if(f.c[k].v==gv)return{corr[fi].outV[k],corr[fi].outVT[k]}; return{0,0};
    };
    for(auto&kv:edgeFaces){
        if(kv.second.size()!=2) continue; // true/open mesh boundaries are free, not proposed as seams
        int f0=kv.second[0], f1=kv.second[1]; if(!corr[f0].ok||!corr[f1].ok) continue;
        auto a0=endpointData(f0,kv.first.a), a1=endpointData(f1,kv.first.a);
        auto b0=endpointData(f0,kv.first.b), b1=endpointData(f1,kv.first.b);
        bool splitV=(a0.first!=a1.first)||(b0.first!=b1.first);
        bool splitVT=(a0.second>0&&a1.second>0&&a0.second!=a1.second)||(b0.second>0&&b1.second>0&&b0.second!=b1.second);
        if(splitV||splitVT) seams.insert(kv.first);
    }
    return true;
}

static fs::path findResultObj(const fs::path& root, const std::string& token) {
    fs::path fallback;
    if(!fs::exists(root)) return {};
    for(auto it=fs::recursive_directory_iterator(root); it!=fs::recursive_directory_iterator(); ++it){
        if(!it->is_regular_file()) continue;
        if(it->path().filename()=="finalResult_mesh.obj") {
            if(it->path().parent_path().string().find(token)!=std::string::npos) return it->path();
            fallback=it->path();
        }
    }
    return fallback;
}

static int runOptCuts(const fs::path& exe, const fs::path& inputObj, const fs::path& runDir, double bound, int initialCut, const std::string& token, fs::path& resultObj, fs::path& logPath) {
    fs::create_directories(runDir);
    logPath=runDir/"optcuts.log";
    fs::path old=fs::current_path(); fs::current_path(runDir);
    std::ostringstream cmd;
    cmd<<quote(exe)<<" 100 "<<quote(inputObj)<<" 0.999 1 0 "<<std::setprecision(8)<<bound<<" 1 "<<initialCut<<" "<<token
       <<" > "<<quote(logPath)<<" 2>&1";
    int rc=std::system(cmd.str().c_str());
    fs::current_path(old);
    resultObj=findResultObj(runDir/"output",token);
    return rc;
}

static double parseBound(const std::string&s){try{return std::stod(s);}catch(...){return 5.5;}}

int main(int argc,char**argv){
    if(argc<4){
        std::cerr<<"RotateUV Native Auto Seam worker\nUsage: RotateUV_AutoSeam.exe input.obj output.seams distortionBound [initialCut]\n";
        return 2;
    }
    fs::path inputPath=fs::absolute(argv[1]); fs::path outputPath=fs::absolute(argv[2]);
    double bound=parseBound(argv[3]); if(bound<=4.0) bound=4.05; int initialCut=(argc>=5?std::atoi(argv[4]):1); if(initialCut!=0&&initialCut!=1) initialCut=1;
    fs::path exeDir=fs::absolute(fs::path(argv[0])).parent_path(); fs::path optExe=exeDir/"OptCuts_bin.exe";
    if(!fs::exists(optExe)){std::cerr<<"OptCuts_bin.exe not found beside worker: "<<optExe<<"\n";return 3;}
    ObjMesh full; std::string err; if(!readTriObj(inputPath,full,err)){std::cerr<<err<<"\n";return 4;}
    auto comps=faceComponents(full); std::set<EdgeKey> globalSeams; int failed=0,totalUnmatched=0;
    auto stamp=std::chrono::high_resolution_clock::now().time_since_epoch().count(); fs::path baseRun=inputPath.parent_path()/("rotateuv_optcuts_"+std::to_string(stamp)); fs::create_directories(baseRun);
    for(size_t ci=0;ci<comps.size();++ci){
        fs::path compDir=baseRun/("component_"+std::to_string(ci+1)); fs::create_directories(compDir); ComponentExport ce; std::string e;
        fs::path compObj=compDir/"input.obj"; if(!writeComponentObj(full,comps[ci],compObj,ce,e)){std::cerr<<"Component export failed: "<<e<<"\n";failed++;continue;}
        std::string token="ruv_"+std::to_string(stamp)+"_c"+std::to_string(ci+1); fs::path resultObj,log;
        int rc=runOptCuts(optExe,compObj,compDir,bound,initialCut,token,resultObj,log);
        if(rc!=0||resultObj.empty()||!fs::exists(resultObj)){
            std::cerr<<"OptCuts failed on component "<<(ci+1)<<" (exit "<<rc<<"). Log: "<<log<<"\n";failed++;continue;
        }
        ObjMesh inComp,outComp; if(!readTriObj(compObj,inComp,e)){std::cerr<<e<<"\n";failed++;continue;} if(!readTriObj(resultObj,outComp,e)){std::cerr<<"Result parse failed: "<<e<<"\n";failed++;continue;}
        std::set<EdgeKey> localSeams; int unmatched=0; if(!deriveSeams(inComp,outComp,localSeams,unmatched,e)){std::cerr<<"Seam extraction failed: "<<e<<"\n";failed++;continue;} totalUnmatched+=unmatched;
        for(auto&s:localSeams){ if(s.a>0&&s.b>0&&s.a<(int)ce.localToGlobal.size()&&s.b<(int)ce.localToGlobal.size()) globalSeams.insert(EdgeKey(ce.localToGlobal[s.a],ce.localToGlobal[s.b])); }
    }
    std::ofstream out(outputPath); if(!out){std::cerr<<"Cannot create seam output file.\n";return 6;}
    out<<"RUVSEAM 1\n";
    out<<"COMPONENTS "<<comps.size()<<"\n";
    out<<"FAILED_COMPONENTS "<<failed<<"\n";
    out<<"UNMATCHED_TRIANGLES "<<totalUnmatched<<"\n";
    out<<"DISTORTION_BOUND "<<std::setprecision(8)<<bound<<"\n";
    out<<"SEAMS "<<globalSeams.size()<<"\n";
    for(auto&s:globalSeams) out<<"SEAM "<<s.a<<" "<<s.b<<"\n";
    out<<"END\n";
    out.close();
    std::error_code ec; fs::remove_all(baseRun,ec);
    if(failed==(int)comps.size()) return 7;
    std::cout<<"RotateUV Auto Seam: "<<globalSeams.size()<<" seam edges from "<<comps.size()<<" component(s); failed="<<failed<<" unmatched="<<totalUnmatched<<"\n";
    return 0;
}
