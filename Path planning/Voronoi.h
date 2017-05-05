// Voronoi.h: interface for the CVoronoi class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_VORONOI_H__B941A16F_4210_4017_9C6A_AEB2389C9F06__INCLUDED_)
#define AFX_VORONOI_H__B941A16F_4210_4017_9C6A_AEB2389C9F06__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <math.h>


#ifndef NULL
#define NULL 0
#endif
#define LE 0
#define RE 1
#define DELETED -2

struct VonoroiPoint
{
   double x,y;
};
struct Site
{
   struct	VonoroiPoint	coord;
   int		sitenbr;       // site number = index in the array 'sites'
   int		refcnt;
};

struct Freenode
{
   struct Freenode *nextfree;
};
struct Freelist 
{
   struct Freenode *head;
   int nodesize;
   char* MemBlock;
};
class Memory
{
   int total_alloc;
   class CVoronoi* Parent;
public:
   Memory(class CVoronoi* parent);
   void freeinit(struct Freelist* fl, int size);
   void makefree(struct Freenode *curr, struct Freelist *fl);
   char *getfree(struct	Freelist *fl);
   char *myalloc(unsigned int n);
   void myfree(struct Freelist* fl);
};
struct Edge
{
   double	a,b,c;
   struct	Site 	*ep[2];
   struct	Site	*reg[2];
   int		edgenbr;
};
struct Halfedge
{
   struct Halfedge *ELleft;
   struct Halfedge *ELright;
   struct Edge	*ELedge;
   int ELrefcnt;
   char ELpm;
   struct Site *vertex;
   double ystar;
   struct Halfedge *PQnext;
};   
class Sites
{
   class CVoronoi* Parent;
public:
	int ReadSites(double* Data);
   double xmin, xmax, ymin, ymax, deltax, deltay;
   struct	Site	*sites;
   int		nsites;
   int		siteidx;
   int  	sqrt_nsites;
   int		nvertices;
   struct 	Freelist sfl;
   struct	Site	*bottomsite;

   struct Site* nextone();
   Sites(class CVoronoi* parent);
   ~Sites();
};


class Heap
{
   int PQcount;
   int PQmin;
   int PQhashsize;
   struct Halfedge *PQhash;

   int PQbucket(struct Halfedge *he);
   class CVoronoi* Parent;

public:
   Heap(class CVoronoi* parent);
   ~Heap();
   void PQinsert(struct Halfedge *he, struct Site *v, double offset);
   void PQdelete(struct Halfedge *he);
   int PQempty();
   struct VonoroiPoint PQ_min();
   struct Halfedge *PQextractmin();
};

class EdgeList
{
   int ntry;
   int totalsearch;
   int ELhashsize;
   struct Freelist hfl;
   struct Halfedge** ELhash;

   struct Halfedge *ELgethash(int b);
   class CVoronoi* Parent;

public:
   struct Halfedge* ELleftend;
   struct Halfedge* ELrightend;

   EdgeList(class CVoronoi* parent);
   ~EdgeList();
   struct Halfedge*  HEcreate(struct Edge *e, int pm);
   struct Halfedge*  ELleftbnd(struct VonoroiPoint *p, double xmin);
   struct Halfedge*  ELright(struct Halfedge *he);
   struct Halfedge*  ELleft(struct Halfedge *he);
   void  ELinsert(struct Halfedge *lb, struct Halfedge *anew);
   void  ELdelete(struct Halfedge *he);
   struct Site*   leftreg(struct Halfedge *he);
   struct Site*   rightreg(struct Halfedge *he);
   int right_of(struct Halfedge *el, struct VonoroiPoint *p);
};   

class Output
{
   class CVoronoi* Parent;
   CDC* pDC;

public:
   double pxmin;
   double pxmax;
   double pymin;
   double pymax;
   double cradius;
   CList<Site, Site&> ListSites;
   CList<Site, Site&> ListVertices;
   CList<Edge, Edge&> ListLines;
   CList<Edge, Edge&> ListEdges;


   Output(class CVoronoi* parent, CDC* pDC);
   int clip_line(struct Edge *e);
   void out_bisector(struct Edge *e);
   void out_ep(struct Edge *e);
   void out_vertex(struct Site *v);
   void out_site(struct Site *s);
   void out_triple(struct Site *s1, struct Site *s2, struct Site *s3);
};

class Geometry
{
   struct Freelist efl;
   int sqrt_nsites;
   int nvertices;
   double deltax, deltay;
   int nedges;

   class CVoronoi* Parent;

public:
   Geometry(class CVoronoi* parent);
   ~Geometry();
   struct Edge *bisect(struct	Site *s1, struct Site *s2);
   struct Site *intersect(struct Halfedge *el1, struct Halfedge *el2);
   void ref(struct Site *v);
   void deref(struct	Site *v);
   void endpoint(struct Edge *e, int	lr, struct Site *s);
   double dist(struct Site *s, struct Site *t);
   void makevertex(struct Site *v);
};

class CVoronoi  
{
	int Calculate();

public:
	CList<Edge, Edge&>* GetEdges();
	CList<Edge, Edge&>* GetLines();
	CList<Site, Site&>* GetVertices();
	Site* GetSites();
	int DrawEdges(CDC* pDC, int tri, int pol);

   int triangulate, plot, debug;
   class Memory* MyMemory;
   class Output* MyOutput;
   class Sites* MySites;
   class Heap* MyHeap;
   class EdgeList* MyEdgeList;
   class Geometry* MyGeometry;
   
	int SetPoints(double* data);

	CVoronoi();
	virtual ~CVoronoi();

};

#endif // !defined(AFX_VORONOI_H__B941A16F_4210_4017_9C6A_AEB2389C9F06__INCLUDED_)
