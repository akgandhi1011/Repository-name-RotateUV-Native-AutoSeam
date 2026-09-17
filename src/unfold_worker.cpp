#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

struct Vec2 { double x=0.0, y=0.0; };
struct Vec3 { double x=0.0, y=0.0, z=0.0; };
static Vec3 operator+(const Vec3&a,const Vec3&b){ return {a.x+b.x,a.y+b.y,a.z+b.z}; }
static Vec3 operator-(const Vec3&a,const Vec3&b){ return {a.x-b.x,a.y-b.y,a.z-b.z}; }
static Vec3 operator*(const Vec3&a,double s){ return {a.x*s,a.y*s,a.z*s}; }
static Vec3 operator/(const Vec3&a,double s){ return s!=0.0?a*(1.0/s):Vec3{}; }
static double dot(const Vec3&a,const Vec3&b){ return a.x*b.x+a.y*b.y+a.z*b.z; }
static Vec3 cross(const Vec3&a,const Vec3&b){ return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
static double len2(const Vec3&a){ return dot(a,a); }
static double len(const Vec3&a){ return std::sqrt(len2(a)); }
static Vec3 norm(const Vec3&a){ const double l=len(a); return l>1e-15?a/l:Vec3{}; }
static double clampd(double v,double lo,double hi){ return std::max(lo,std::min(hi,v)); }

struct EdgeKey {
    int a=0,b=0;
    EdgeKey()=default;
    EdgeKey(int x,int y){ if(x<y){a=x;b=y;} else {a=y;b=x;} }
    bool operator==(const EdgeKey&o)const{return a==o.a&&b==o.b;}
    bool operator<(const EdgeKey&o)const{return a<o.a||(a==o.a&&b<o.b);}
};
struct EdgeHash {
    size_t operator()(const EdgeKey&e)const noexcept {
        return (static_cast<size_t>(static_cast<uint32_t>(e.a))<<32)^static_cast<uint32_t>(e.b);
    }
};

struct Face { std::vector<int> v; };
struct InputMesh {
    std::vector<Vec3> vertices; // geometry vertex index = vector index + 1
    std::vector<Face> faces;
    std::set<EdgeKey> seams;
};

static bool readInput(const std::string&path, InputMesh&m, std::string&err){
    std::ifstream in(path);
    if(!in){ err="Cannot open native unfold input."; return false; }
    std::string line;
    if(!std::getline(in,line) || line!="RUVUNFOLD 1") { err="Invalid RUVUNFOLD header."; return false; }
    int expectedVerts=-1, expectedFaces=-1, expectedSeams=-1;
    while(std::getline(in,line)){
        if(line.empty()) continue;
        std::istringstream ss(line); std::string tag; ss>>tag;
        if(tag=="VERTICES"){
            ss>>expectedVerts; if(expectedVerts<3){err="Invalid vertex count.";return false;}
            m.vertices.reserve(expectedVerts);
            for(int i=0;i<expectedVerts;i++){
                if(!std::getline(in,line)){err="Unexpected EOF in VERTICES.";return false;}
                std::istringstream vs(line); Vec3 p; if(!(vs>>p.x>>p.y>>p.z)){err="Malformed vertex record.";return false;} m.vertices.push_back(p);
            }
        }else if(tag=="FACES"){
            ss>>expectedFaces; if(expectedFaces<1){err="Invalid face count.";return false;}
            m.faces.resize(expectedFaces);
            for(int i=0;i<expectedFaces;i++){
                if(!std::getline(in,line)){err="Unexpected EOF in FACES.";return false;}
                std::istringstream fs(line); std::string ft; int faceIndex=0,n=0; fs>>ft>>faceIndex>>n;
                if(ft!="FACE" || faceIndex!=i+1 || n<3){err="Malformed FACE record.";return false;}
                m.faces[i].v.resize(n);
                for(int k=0;k<n;k++){
                    if(!(fs>>m.faces[i].v[k])){err="FACE record is missing geometry vertex indices.";return false;}
                    if(m.faces[i].v[k]<1 || m.faces[i].v[k]>(int)m.vertices.size()){err="FACE geometry vertex index out of range.";return false;}
                }
            }
        }else if(tag=="SEAMS"){
            ss>>expectedSeams; if(expectedSeams<0){err="Invalid seam count.";return false;}
            for(int i=0;i<expectedSeams;i++){
                if(!std::getline(in,line)){err="Unexpected EOF in SEAMS.";return false;}
                std::istringstream es(line); std::string st; int a=0,b=0; es>>st>>a>>b;
                if(st!="SEAM" || a<1 || b<1 || a>(int)m.vertices.size() || b>(int)m.vertices.size() || a==b){err="Malformed SEAM record.";return false;}
                m.seams.insert(EdgeKey(a,b));
            }
        }else if(tag=="END") break;
    }
    if((int)m.vertices.size()!=expectedVerts || (int)m.faces.size()!=expectedFaces){err="Native unfold input is incomplete.";return false;}
    return true;
}

struct DSU {
    std::vector<int> p,r;
    explicit DSU(int n=0):p(n),r(n,0){std::iota(p.begin(),p.end(),0);} 
    int find(int x){return p[x]==x?x:p[x]=find(p[x]);}
    void unite(int a,int b){a=find(a);b=find(b);if(a==b)return;if(r[a]<r[b])std::swap(a,b);p[b]=a;if(r[a]==r[b])r[a]++;}
};

struct HalfEdge { int face=0; int corner=0; int a=0; int b=0; };
struct CutMesh {
    std::vector<int> faceCornerStart;
    std::vector<int> cornerCutVertex;
    std::vector<int> cutGeomVertex;
    std::vector<std::vector<int>> faceAdj;
    int cutVertexCount=0;
};

static CutMesh buildCutMesh(const InputMesh&m){
    CutMesh c; const int nf=(int)m.faces.size(); c.faceCornerStart.resize(nf+1,0);
    for(int f=0;f<nf;f++) c.faceCornerStart[f+1]=c.faceCornerStart[f]+(int)m.faces[f].v.size();
    const int nc=c.faceCornerStart[nf]; DSU dsu(nc);
    std::unordered_map<EdgeKey,std::vector<HalfEdge>,EdgeHash> emap;
    emap.reserve((size_t)nc*2);
    for(int f=0;f<nf;f++){
        const int n=(int)m.faces[f].v.size();
        for(int k=0;k<n;k++){
            const int a=m.faces[f].v[k], b=m.faces[f].v[(k+1)%n];
            emap[EdgeKey(a,b)].push_back({f,k,a,b});
        }
    }
    c.faceAdj.assign(nf,{});
    for(auto&kv:emap){
        const EdgeKey&e=kv.first; auto&hs=kv.second;
        if(hs.size()!=2 || m.seams.count(e)) continue;
        const HalfEdge&h0=hs[0]; const HalfEdge&h1=hs[1];
        const int n0=(int)m.faces[h0.face].v.size(), n1=(int)m.faces[h1.face].v.size();
        const int c0=c.faceCornerStart[h0.face]+h0.corner;
        const int c0n=c.faceCornerStart[h0.face]+((h0.corner+1)%n0);
        const int c1=c.faceCornerStart[h1.face]+h1.corner;
        const int c1n=c.faceCornerStart[h1.face]+((h1.corner+1)%n1);
        if(h0.a==h1.a){ dsu.unite(c0,c1); dsu.unite(c0n,c1n); }
        else { dsu.unite(c0,c1n); dsu.unite(c0n,c1); }
        c.faceAdj[h0.face].push_back(h1.face); c.faceAdj[h1.face].push_back(h0.face);
    }
    c.cornerCutVertex.resize(nc,-1);
    std::unordered_map<int,int> rootToCut; rootToCut.reserve(nc);
    for(int f=0;f<nf;f++){
        const int n=(int)m.faces[f].v.size();
        for(int k=0;k<n;k++){
            const int ci=c.faceCornerStart[f]+k; const int root=dsu.find(ci);
            auto it=rootToCut.find(root); int cv;
            if(it==rootToCut.end()){
                cv=(int)rootToCut.size(); rootToCut[root]=cv; c.cutGeomVertex.push_back(m.faces[f].v[k]);
            }else cv=it->second;
            c.cornerCutVertex[ci]=cv;
        }
    }
    c.cutVertexCount=(int)rootToCut.size(); return c;
}

struct Tri { int a=0,b=0,c=0; };
struct Chart {
    std::vector<int> faces;
    std::vector<int> cutVerts;
    std::vector<Tri> tris;
    std::vector<Vec2> uv;
    std::unordered_map<int,int> cutToLocal;
    int flips=0;
};

static std::vector<Chart> buildCharts(const InputMesh&m,const CutMesh&c){
    const int nf=(int)m.faces.size(); std::vector<char>seen(nf,0); std::vector<Chart>charts;
    for(int seed=0;seed<nf;seed++) if(!seen[seed]){
        Chart ch; std::queue<int>q; q.push(seed); seen[seed]=1;
        while(!q.empty()){int f=q.front();q.pop();ch.faces.push_back(f);for(int n:c.faceAdj[f])if(!seen[n]){seen[n]=1;q.push(n);}}
        std::set<int> cvs;
        for(int f:ch.faces){
            const int n=(int)m.faces[f].v.size();
            for(int k=0;k<n;k++) cvs.insert(c.cornerCutVertex[c.faceCornerStart[f]+k]);
        }
        ch.cutVerts.assign(cvs.begin(),cvs.end());
        ch.cutToLocal.reserve(ch.cutVerts.size()*2);
        for(int i=0;i<(int)ch.cutVerts.size();i++) ch.cutToLocal[ch.cutVerts[i]]=i;
        for(int f:ch.faces){
            const int n=(int)m.faces[f].v.size();
            const int c0=ch.cutToLocal[c.cornerCutVertex[c.faceCornerStart[f]]];
            for(int k=1;k+1<n;k++){
                const int c1=ch.cutToLocal[c.cornerCutVertex[c.faceCornerStart[f]+k]];
                const int c2=ch.cutToLocal[c.cornerCutVertex[c.faceCornerStart[f]+k+1]];
                if(c0!=c1&&c1!=c2&&c2!=c0) ch.tris.push_back({c0,c1,c2});
            }
        }
        charts.push_back(std::move(ch));
    }
    return charts;
}

static Vec3 chartPos(const InputMesh&m,const CutMesh&c,const Chart&ch,int local){
    const int cv=ch.cutVerts[local]; const int gv=c.cutGeomVertex[cv]; return m.vertices[gv-1];
}

static bool planarProject(const InputMesh&m,const CutMesh&c,Chart&ch){
    if(ch.tris.empty()||ch.cutVerts.size()<3)return false;
    Vec3 nsum{}; double areaSum=0.0;
    for(const auto&t:ch.tris){
        Vec3 a=chartPos(m,c,ch,t.a),b=chartPos(m,c,ch,t.b),d=chartPos(m,c,ch,t.c);
        Vec3 cr=cross(b-a,d-a); double twice=len(cr); if(twice<1e-14)continue; nsum=nsum+cr; areaSum+=0.5*twice;
    }
    Vec3 n=norm(nsum); if(len2(n)<1e-12||areaSum<1e-14)return false;
    double worst=0.0;
    for(const auto&t:ch.tris){
        Vec3 a=chartPos(m,c,ch,t.a),b=chartPos(m,c,ch,t.b),d=chartPos(m,c,ch,t.c); Vec3 tn=norm(cross(b-a,d-a)); if(len2(tn)<1e-12)continue;
        const double ang=std::acos(clampd(std::abs(dot(tn,n)),-1.0,1.0))*57.29577951308232; worst=std::max(worst,ang);
    }
    if(worst>3.0)return false;
    // Stable basis: longest vector from centroid projected to plane.
    Vec3 cen{}; for(int i=0;i<(int)ch.cutVerts.size();i++)cen=cen+chartPos(m,c,ch,i); cen=cen/(double)ch.cutVerts.size();
    Vec3 e1{}; double best=-1.0;
    for(int i=0;i<(int)ch.cutVerts.size();i++){Vec3 v=chartPos(m,c,ch,i)-cen;v=v-n*dot(v,n);double q=len2(v);if(q>best){best=q;e1=v;}}
    e1=norm(e1); if(len2(e1)<1e-12){e1=norm(cross(n,{0,0,1}));if(len2(e1)<1e-12)e1=norm(cross(n,{0,1,0}));}
    Vec3 e2=norm(cross(n,e1)); ch.uv.resize(ch.cutVerts.size());
    for(int i=0;i<(int)ch.cutVerts.size();i++){Vec3 d=chartPos(m,c,ch,i)-cen;ch.uv[i]={dot(d,e1),dot(d,e2)};}
    return true;
}

struct Row { std::vector<std::pair<int,double>> a; double rhs=0.0; };

static bool pcgNormal(const std::vector<Row>&rows,int n,std::vector<double>&x){
    if(n<=0){x.clear();return true;}
    std::vector<double>b(n,0.0),diag(n,1e-12);
    for(const auto&r:rows){for(const auto&ic:r.a){b[ic.first]+=ic.second*r.rhs;diag[ic.first]+=ic.second*ic.second;}}
    auto apply=[&](const std::vector<double>&vin,std::vector<double>&out){
        out.assign(n,0.0);
        for(const auto&r:rows){double s=0.0;for(const auto&ic:r.a)s+=ic.second*vin[ic.first];for(const auto&ic:r.a)out[ic.first]+=ic.second*s;}
        const double reg=1e-12;for(int i=0;i<n;i++)out[i]+=reg*vin[i];
    };
    x.assign(n,0.0); std::vector<double>r=b,z(n),p(n),Ap;
    for(int i=0;i<n;i++)z[i]=r[i]/diag[i]; p=z;
    double rz=0.0,bn=0.0; for(int i=0;i<n;i++){rz+=r[i]*z[i];bn+=b[i]*b[i];}
    if(bn<1e-30)return true;
    const double target=std::max(1e-18,bn*1e-20); const int maxIter=std::min(12000,std::max(800,n*8));
    for(int it=0;it<maxIter;it++){
        apply(p,Ap); double pAp=0.0;for(int i=0;i<n;i++)pAp+=p[i]*Ap[i]; if(std::abs(pAp)<1e-30)break;
        double alpha=rz/pAp;for(int i=0;i<n;i++){x[i]+=alpha*p[i];r[i]-=alpha*Ap[i];}
        double rn=0.0;for(double v:r)rn+=v*v;if(rn<=target)return true;
        for(int i=0;i<n;i++)z[i]=r[i]/diag[i];double rz2=0.0;for(int i=0;i<n;i++)rz2+=r[i]*z[i];
        if(std::abs(rz)<1e-30)break;double beta=rz2/rz;for(int i=0;i<n;i++)p[i]=z[i]+beta*p[i];rz=rz2;
    }
    // Accept a finite approximate solution; simple standard meshes converge well before this.
    for(double v:x)if(!std::isfinite(v))return false;return true;
}

static int farthestEuclidean(const InputMesh&m,const CutMesh&c,const Chart&ch,int seed){
    Vec3 p=chartPos(m,c,ch,seed);int best=seed;double bd=-1.0;
    for(int i=0;i<(int)ch.cutVerts.size();i++){double d=len2(chartPos(m,c,ch,i)-p);if(d>bd){bd=d;best=i;}}return best;
}

static bool lscmSolve(const InputMesh&m,const CutMesh&c,Chart&ch){
    const int n=(int)ch.cutVerts.size(); if(n<3||ch.tris.empty())return false;
    int p0=farthestEuclidean(m,c,ch,0);int p1=farthestEuclidean(m,c,ch,p0);if(p0==p1)return false;
    double pinDist=len(chartPos(m,c,ch,p1)-chartPos(m,c,ch,p0));if(pinDist<1e-8)pinDist=1.0;
    std::vector<char>fixed(2*n,0);std::vector<double>fixedVal(2*n,0.0);
    fixed[p0]=fixed[p0+n]=fixed[p1]=fixed[p1+n]=1; fixedVal[p1]=pinDist;
    std::vector<int>freeIndex(2*n,-1);int freeN=0;for(int i=0;i<2*n;i++)if(!fixed[i])freeIndex[i]=freeN++;
    std::vector<Row>rows;rows.reserve(ch.tris.size()*2);
    for(const auto&t:ch.tris){
        Vec3 P0=chartPos(m,c,ch,t.a),P1=chartPos(m,c,ch,t.b),P2=chartPos(m,c,ch,t.c);
        const double l01=len(P1-P0),l02=len(P2-P0),l12=len(P2-P1);if(l01<1e-12)continue;
        double x2=(l02*l02+l01*l01-l12*l12)/(2.0*l01);double y2sq=std::max(0.0,l02*l02-x2*x2);double y2=std::sqrt(y2sq);if(y2<1e-12)continue;
        const double D=l01*y2; const double gx[3]={-y2/D,y2/D,0.0}; const double gy[3]={(x2-l01)/D,-x2/D,l01/D};
        const int vi[3]={t.a,t.b,t.c};const double w=std::sqrt(std::max(1e-16,0.5*D));
        Row r1,r2;
        auto add=[&](Row&row,int full,double coeff){coeff*=w;if(fixed[full])row.rhs-=coeff*fixedVal[full];else row.a.push_back({freeIndex[full],coeff});};
        for(int j=0;j<3;j++){
            add(r1,vi[j],gx[j]); add(r1,vi[j]+n,-gy[j]);
            add(r2,vi[j],gy[j]); add(r2,vi[j]+n,gx[j]);
        }
        rows.push_back(std::move(r1));rows.push_back(std::move(r2));
    }
    if(rows.size()<2)return false;std::vector<double>x;if(!pcgNormal(rows,freeN,x))return false;
    ch.uv.assign(n,{});
    for(int i=0;i<n;i++){
        auto val=[&](int full)->double{return fixed[full]?fixedVal[full]:x[freeIndex[full]];};
        ch.uv[i]={val(i),val(i+n)};
        if(!std::isfinite(ch.uv[i].x)||!std::isfinite(ch.uv[i].y))return false;
    }
    // Prefer a globally consistent orientation. This does not hide local flips; it only mirrors the whole chart if needed.
    int pos=0,neg=0;for(const auto&t:ch.tris){Vec2 a=ch.uv[t.a],b=ch.uv[t.b],d=ch.uv[t.c];double A=(b.x-a.x)*(d.y-a.y)-(b.y-a.y)*(d.x-a.x);if(A>1e-12)pos++;else if(A<-1e-12)neg++;}
    if(neg>pos)for(auto&uv:ch.uv)uv.y=-uv.y;
    ch.flips=std::min(pos,neg);return true;
}

static void fallbackProject(const InputMesh&m,const CutMesh&c,Chart&ch){
    // PCA-lite fallback: choose the two longest bbox axes in world space.
    Vec3 mn{1e100,1e100,1e100},mx{-1e100,-1e100,-1e100};
    for(int i=0;i<(int)ch.cutVerts.size();i++){Vec3 p=chartPos(m,c,ch,i);mn.x=std::min(mn.x,p.x);mn.y=std::min(mn.y,p.y);mn.z=std::min(mn.z,p.z);mx.x=std::max(mx.x,p.x);mx.y=std::max(mx.y,p.y);mx.z=std::max(mx.z,p.z);}    
    std::array<std::pair<double,int>,3> axes={{{mx.x-mn.x,0},{mx.y-mn.y,1},{mx.z-mn.z,2}}};std::sort(axes.begin(),axes.end(),[](auto&a,auto&b){return a.first>b.first;});
    ch.uv.resize(ch.cutVerts.size());
    auto coord=[](const Vec3&p,int a){return a==0?p.x:(a==1?p.y:p.z);};
    for(int i=0;i<(int)ch.cutVerts.size();i++){Vec3 p=chartPos(m,c,ch,i);ch.uv[i]={coord(p,axes[0].second),coord(p,axes[1].second)};}
}

static void packCharts(std::vector<Chart>&charts){
    double totalArea=0.0,maxH=0.0;
    struct B{double minx,maxx,miny,maxy;};std::vector<B>bb(charts.size());
    for(size_t ci=0;ci<charts.size();ci++){
        auto&ch=charts[ci];double minx=1e100,miny=1e100,maxx=-1e100,maxy=-1e100;
        for(auto&u:ch.uv){minx=std::min(minx,u.x);miny=std::min(miny,u.y);maxx=std::max(maxx,u.x);maxy=std::max(maxy,u.y);}if(ch.uv.empty()){minx=miny=0;maxx=maxy=1;}
        bb[ci]={minx,maxx,miny,maxy};totalArea+=(maxx-minx)*(maxy-miny);maxH=std::max(maxH,maxy-miny);
    }
    double targetW=std::max(1.0,std::sqrt(std::max(1e-12,totalArea))*1.7);double pad=std::max(1e-5,maxH*0.04);double x=0.0,y=0.0,rowH=0.0;
    for(size_t ci=0;ci<charts.size();ci++){
        double w=bb[ci].maxx-bb[ci].minx,h=bb[ci].maxy-bb[ci].miny;if(x>0&&x+w>targetW){x=0;y+=rowH+pad;rowH=0;}
        for(auto&u:charts[ci].uv){u.x+=x-bb[ci].minx;u.y+=y-bb[ci].miny;}x+=w+pad;rowH=std::max(rowH,h);
    }
    double minx=1e100,miny=1e100,maxx=-1e100,maxy=-1e100;for(auto&ch:charts)for(auto&u:ch.uv){minx=std::min(minx,u.x);miny=std::min(miny,u.y);maxx=std::max(maxx,u.x);maxy=std::max(maxy,u.y);}
    double w=maxx-minx,h=maxy-miny,scale=0.96/std::max(1e-9,std::max(w,h));for(auto&ch:charts)for(auto&u:ch.uv){u.x=0.02+(u.x-minx)*scale;u.y=0.02+(u.y-miny)*scale;}
}

static bool writeOutput(const std::string&path,const InputMesh&m,const CutMesh&c,const std::vector<Chart>&charts,std::string&err){
    std::vector<int> cutChart(c.cutVertexCount,-1),cutLocal(c.cutVertexCount,-1);int flips=0;
    for(int ci=0;ci<(int)charts.size();ci++){flips+=charts[ci].flips;for(int li=0;li<(int)charts[ci].cutVerts.size();li++){int cv=charts[ci].cutVerts[li];cutChart[cv]=ci;cutLocal[cv]=li;}}
    std::ofstream out(path);if(!out){err="Cannot create native unfold result.";return false;}
    out<<"RUVUV 1\nSTATUS OK\nCHARTS "<<charts.size()<<"\nFLIPS "<<flips<<"\nFACES "<<m.faces.size()<<"\n";
    out<<std::setprecision(17);
    for(int f=0;f<(int)m.faces.size();f++){
        const int n=(int)m.faces[f].v.size();out<<"FACE "<<(f+1)<<" "<<n<<"\n";
        for(int k=0;k<n;k++){
            const int cv=c.cornerCutVertex[c.faceCornerStart[f]+k];const int ci=cutChart[cv],li=cutLocal[cv];
            if(ci<0||li<0){err="Internal chart mapping failure.";return false;}const Vec2&uv=charts[ci].uv[li];
            out<<"UV "<<(k+1)<<" "<<uv.x<<" "<<uv.y<<" "<<cv<<"\n";
        }
        out<<"END_FACE\n";
    }
    out<<"END\n";return true;
}

int main(int argc,char**argv){
    if(argc<3){
        std::cerr<<"RotateUV Native Unfold V1 - seam-constrained LSCM\nUsage: RotateUV_Unfold.exe input.ruvu output.ruvuv\n";return 2;
    }
    InputMesh m;std::string err;if(!readInput(argv[1],m,err)){std::cerr<<err<<"\n";return 4;}
    CutMesh c=buildCutMesh(m);auto charts=buildCharts(m,c);if(charts.empty()){std::cerr<<"No UV charts could be created.\n";return 5;}
    int fallbackCount=0,totalFlips=0;
    for(auto&ch:charts){bool ok=planarProject(m,c,ch);if(!ok)ok=lscmSolve(m,c,ch);if(!ok){fallbackProject(m,c,ch);fallbackCount++;}totalFlips+=ch.flips;}
    packCharts(charts);if(!writeOutput(argv[2],m,c,charts,err)){std::cerr<<err<<"\n";return 6;}
    std::cout<<"RotateUV Native Unfold V1: charts="<<charts.size()<<" cutVerts="<<c.cutVertexCount<<" seams="<<m.seams.size()<<" fallbacks="<<fallbackCount<<" flips="<<totalFlips<<"\n";
    return 0;
}
