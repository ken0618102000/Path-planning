// Voronoi.cpp: implementation of the CVoronoi class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Voronoi.h"




#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

#include <math.h>
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

/* sort sites on y, then x, coord */
int scomp(const void *s1, const void *s2)
{
   struct Point
   {
      double x,y;
   };

   if(((struct Point *)s1) -> y < ((struct Point *)s2) -> y) return(-1);
	if(((struct Point *)s1) -> y > ((struct Point *)s2) -> y) return(1);
	if(((struct Point *)s1) -> x < ((struct Point *)s2) -> x) return(-1);
	if(((struct Point *)s1) -> x > ((struct Point *)s2) -> x) return(1);
	return(0);
}

CVoronoi::CVoronoi()
{
   triangulate = 0; plot = 0; debug = 1;

   MySites=NULL;
   MyMemory=NULL;
   MyOutput=NULL;
   MyHeap=NULL;
   MyEdgeList=NULL;
   MyGeometry=NULL;
}

CVoronoi::~CVoronoi()
{
   if(MyMemory) delete MyMemory;
   if(MySites) delete MySites;
   if(MyOutput) delete MyOutput;
   if(MyHeap) delete MyHeap;
   if(MyEdgeList) delete MyEdgeList;
   if(MyGeometry) delete MyGeometry;

}

// Data format: [count][x0][y0][x1][y1]....
int CVoronoi::SetPoints(double* Data)
{
   if(MyMemory) delete MyMemory;
   if(MySites) delete MySites;
   if(MyOutput) delete MyOutput;
   if(MyHeap) delete MyHeap;
   if(MyEdgeList) delete MyEdgeList;
   if(MyGeometry) delete MyGeometry;

   MySites=new Sites(this);
   MyMemory=new Memory(this);
   MyOutput=NULL;
   MyHeap=NULL;
   MyEdgeList=NULL;
   MyGeometry=NULL;
 
   MyMemory->freeinit(&(MySites->sfl), sizeof(struct Site));

   return(MySites->ReadSites(Data));
}

int CVoronoi::DrawEdges(CDC *pDC, int tri, int pol)
{
   triangulate=tri; 
   plot=pol;
   MyOutput=new Output(this, pDC);
   Calculate();
   return(0);   
}

Site* CVoronoi::GetSites()
{
//   if(!MyOutput)  return(NULL);
//   return &(MyOutput->ListSites);
   return (MySites->sites);
}

CList<Site, Site&>* CVoronoi::GetVertices()
{
   if(!MyOutput)  return(NULL);
   return &(MyOutput->ListVertices);
}

CList<Edge, Edge&>* CVoronoi::GetLines()
{
   if(!MyOutput)  return(NULL);
   return &(MyOutput->ListLines);
}

CList<Edge, Edge&>* CVoronoi::GetEdges()
{
   if(!MyOutput)  return(NULL);
   return &(MyOutput->ListEdges);
}

int CVoronoi::Calculate()
{
   MyHeap=new Heap(this);
   MyEdgeList=new EdgeList(this);
   MyGeometry=new Geometry(this);

   struct Site *newsite, *bot, *top, *temp, *p;
   struct Site *v;
   struct VonoroiPoint newintstar={0L, 0L};
   int pm;
   struct Halfedge *lbnd, *rbnd, *llbnd, *rrbnd, *bisector;
   struct Edge *e;

   MySites->bottomsite=MySites->nextone();
   MyOutput->out_site(MySites->bottomsite);

   newsite = MySites->nextone();

//    vector<vector<Point>> contours;
//    Mat origin_bmp = imread("格點化地圖.bmp", 1);
//    cvtColor(origin_bmp, origin_bmp, CV_BGR2GRAY);
//    findContours(origin_bmp, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);


   while(1)
   {
	   // when not empty
      if(!MyHeap->PQempty()) 
         newintstar = MyHeap->PQ_min();

	  if (newsite != (struct Site *)NULL
		  && (MyHeap->PQempty() || (newsite->coord.y < newintstar.y)
		  || (newsite->coord.y == newintstar.y
		  && newsite->coord.x < newintstar.x) ) )
	  {
		  /* new site is smallest */
		  MyOutput->out_site(newsite);
		  lbnd = MyEdgeList->ELleftbnd(&(newsite->coord), MySites->xmin);
//連通物件測試開始---------------------------------------------








		  rbnd = MyEdgeList->ELright(lbnd);
		  bot = MyEdgeList->rightreg(lbnd);
		  e = MyGeometry->bisect(bot, newsite);
		  bisector = MyEdgeList->HEcreate(e, LE);
		  MyEdgeList->ELinsert(lbnd, bisector);
		  if ((p = MyGeometry->intersect(lbnd, bisector)) != (struct Site *) NULL)
		  {
			  MyHeap->PQdelete(lbnd);
			  MyHeap->PQinsert(lbnd, p, MyGeometry->dist(p, newsite));
		  }
		  lbnd = bisector;
		  bisector = MyEdgeList->HEcreate(e, RE);
		  MyEdgeList->ELinsert(lbnd, bisector);
		  if ((p = MyGeometry->intersect(bisector, rbnd)) != (struct Site *) NULL)
		  {
			  MyHeap->PQinsert(bisector, p, MyGeometry->dist(p, newsite));
		  }
		  newsite = MySites->nextone();
	  }
	  else if (!MyHeap->PQempty())
		  /* intersection is smallest */
	  {
		  lbnd = MyHeap->PQextractmin();
		  llbnd = MyEdgeList->ELleft(lbnd);
		  rbnd = MyEdgeList->ELright(lbnd);
		  rrbnd = MyEdgeList->ELright(rbnd);
		  bot = MyEdgeList->leftreg(lbnd);
		  top = MyEdgeList->rightreg(rbnd);
		  MyOutput->out_triple(bot, top, MyEdgeList->rightreg(lbnd));
		  v = lbnd->vertex;
		  MyGeometry->makevertex(v);
		  MyGeometry->endpoint(lbnd->ELedge, lbnd->ELpm, v);
		  MyGeometry->endpoint(rbnd->ELedge, rbnd->ELpm, v);
		  MyEdgeList->ELdelete(lbnd);
		  MyHeap->PQdelete(rbnd);
		  MyEdgeList->ELdelete(rbnd);
		  pm = LE;
		  if (bot->coord.y > top->coord.y)
		  {
			  // exchange
			  temp = bot;
			  bot = top;
			  top = temp;
			  pm = RE;
		  }
		  e = MyGeometry->bisect(bot, top);
		  bisector = MyEdgeList->HEcreate(e, pm);
		  MyEdgeList->ELinsert(llbnd, bisector);
		  MyGeometry->endpoint(e, RE - pm, v);
		  MyGeometry->deref(v);
		  if ((p = MyGeometry->intersect(llbnd, bisector)) != (struct Site *) NULL)
		  {
			  MyHeap->PQdelete(llbnd);
			  MyHeap->PQinsert(llbnd, p, MyGeometry->dist(p, bot));
		  }
		  if ((p = MyGeometry->intersect(bisector, rrbnd)) != (struct Site *) NULL)
		  {
			  MyHeap->PQinsert(bisector, p, MyGeometry->dist(p, bot));
		  }
	  }
	   else 
         break;
   }

   for(lbnd=MyEdgeList->ELright(MyEdgeList->ELleftend); 
      lbnd != MyEdgeList->ELrightend; lbnd=MyEdgeList->ELright(lbnd))
	{	
      e = lbnd -> ELedge;
		MyOutput->out_ep(e);
	}

   return(0);
}


Sites::Sites(class CVoronoi* parent)
{
   Parent=parent;
   sites=NULL;
}

Sites::~Sites()
{
   if(sites)
      delete [] sites;
}

/* return a single in-storage site */
struct Site* Sites::nextone()
{
   struct Site *s;
   if(siteidx < nsites)
   {	
      s = &sites[siteidx];
	   siteidx++;
	   return(s);
   }
   else
      return( (struct Site *)NULL);
}

int Sites::ReadSites(double *data)
{
   int i;
   nsites=(int)data[0];
   if(sites)
      delete [] sites;
   sites = new struct Site[nsites];
   for(i=0; i<nsites; i++)
   {
      sites[i].coord.x=data[i*2+1];
      sites[i].coord.y=data[i*2+2];
      //sites[i].sitenbr = i;
	   sites[i].refcnt = 0;
   }
   //先比y, 再比x, 由小到大
   qsort(sites, nsites, sizeof *sites, scomp);
   // modify from original order to sorted order
   for(i=0; i<nsites; i++)
      sites[i].sitenbr=i;
   //get xmin, xmax, ymin, ymax
   xmin=sites[0].coord.x; 
   xmax=sites[0].coord.x;
   for(i=1; i<nsites; i++)
   {
	   if (sites[i].coord.x < xmin) xmin = sites[i].coord.x;
	   if (sites[i].coord.x > xmax) xmax = sites[i].coord.x;
   }
   ymin = sites[0].coord.y;
   ymax = sites[nsites-1].coord.y;

   deltax = xmax - xmin;
	deltay = ymax - ymin;

   sqrt_nsites = sqrt((float)nsites+4);
   siteidx = 0;
   return(nsites);
}


Heap::Heap(CVoronoi* parent)
{
   Parent=parent;
   int i;

	PQcount = 0;
	PQmin = 0;
	PQhashsize = 4 * Parent->MySites->sqrt_nsites;
   PQhash=new Halfedge[PQhashsize];
	for(i=0; i<PQhashsize; i++) 
      PQhash[i].PQnext = (struct Halfedge *)NULL;
}

Heap::~Heap()
{
   delete [] PQhash;
}


int Heap::PQbucket(struct Halfedge *he)
{
   int bucket;

   bucket = (int)((he->ystar - Parent->MySites->ymin)/Parent->MySites->deltay * PQhashsize);
   if (bucket<0)
      bucket = 0;
   if (bucket>=PQhashsize)
      bucket = PQhashsize-1 ;
   if (bucket < PQmin)
      PQmin = bucket;
   return(bucket);
}


void Heap::PQinsert(struct Halfedge *he, struct Site *v, double offset)
{
   struct Halfedge *last, *next;

   he -> vertex = v;
   v->refcnt++;
   he -> ystar = (double)(v -> coord.y + offset);
   last = &PQhash[PQbucket(he)];
   while ((next = last -> PQnext) != (struct Halfedge *) NULL &&
         (he -> ystar  > next -> ystar  ||
         (he -> ystar == next -> ystar && v -> coord.x > next->vertex->coord.x)))
	{	last = next;};
   he -> PQnext = last -> PQnext; 
   last -> PQnext = he;
   PQcount += 1;
}

void Heap::PQdelete(struct Halfedge *he)
{
   struct Halfedge *last;

   if(he ->  vertex != (struct Site *) NULL)
   {	last = &PQhash[PQbucket(he)];
	   while (last -> PQnext != he) 
         last = last -> PQnext;
	   last -> PQnext = he -> PQnext;
	   PQcount -= 1;
      he -> vertex->refcnt--;
      if(he -> vertex->refcnt == 0) 
         Parent->MyMemory->makefree((struct Freenode*)(he -> vertex), &(Parent->MySites->sfl));
	   he -> vertex = (struct Site *) NULL;
   }
}


int Heap::PQempty()
{
	return(PQcount==0);
}


struct VonoroiPoint Heap::PQ_min()
{
   struct VonoroiPoint answer;

	while(PQhash[PQmin].PQnext == (struct Halfedge *)NULL)
   {  PQmin++;
   }
	answer.x = PQhash[PQmin].PQnext -> vertex -> coord.x;
	answer.y = PQhash[PQmin].PQnext -> ystar;
	return (answer);
}

struct Halfedge* Heap::PQextractmin()
{
   struct Halfedge *curr;

	curr = PQhash[PQmin].PQnext;
	PQhash[PQmin].PQnext = curr -> PQnext;
	PQcount -= 1;
	return(curr);
}


EdgeList::EdgeList(CVoronoi* parent)
{
   Parent=parent;

	ELhashsize = 2*Parent->MySites->sqrt_nsites;
   ELhash=new Halfedge*[ELhashsize];
   memset(ELhash, 0, ELhashsize*4);

   ELleftend = HEcreate( (struct Edge *)NULL, 0);
	ELrightend = HEcreate( (struct Edge *)NULL, 0);

   ELleftend -> ELleft = (struct Halfedge *)NULL;
	ELleftend -> ELright = ELrightend;
	ELrightend -> ELleft = ELleftend;
	ELrightend -> ELright = (struct Halfedge *)NULL;
	ELhash[0] = ELleftend;
	ELhash[ELhashsize-1] = ELrightend;
}

EdgeList::~EdgeList()
{
   delete [] ELhash;
}



struct Halfedge* EdgeList::HEcreate(struct Edge *e, int pm)
{
   struct Halfedge *answer;
   answer=new Halfedge;
	answer -> ELedge = e;
	answer -> ELpm = pm;
	answer -> PQnext = (struct Halfedge *) NULL;
	answer -> vertex = (struct Site *) NULL;
	answer -> ELrefcnt = 0;
	return(answer);
}


void EdgeList::ELinsert(struct Halfedge *lb, struct Halfedge *anew)
{
	anew -> ELleft = lb;
	anew -> ELright = lb -> ELright;
	(lb -> ELright) -> ELleft = anew;
	lb -> ELright = anew;
}


/* Get entry from hash table, pruning any deleted nodes */
struct Halfedge* EdgeList::ELgethash(int b)
{
   struct Halfedge *he;

	if(b<0 || b>=ELhashsize)
      return((struct Halfedge *) NULL);

	he = ELhash[b]; 
	if(he == (struct Halfedge *) NULL || 
         he->ELedge != (struct Edge *) DELETED ) 
      return (he);

   /* Hash table points to deleted half edge.  Patch as necessary. */
	ELhash[b] = (struct Halfedge *) NULL;
	if(--(he->ELrefcnt) == 0) 
      delete he;
	return ((struct Halfedge *) NULL);
}	

/* returns 1 if p is to right of halfedge e */
int EdgeList::right_of(struct Halfedge *el, struct VonoroiPoint *p)
{
   struct Edge *e;
   struct Site *topsite;
   int right_of_site, above, fast;
   double dxp, dyp, dxs, t1, t2, t3, yl;

   e = el->ELedge;
   topsite = e -> reg[1];
   right_of_site = p->x > topsite->coord.x;
   if(right_of_site && el->ELpm == LE) return(1);
   if(!right_of_site && el->ELpm == RE) return(0);

   if (e->a == 1.0)
   {	
      dyp = p->y - topsite->coord.y;
	   dxp = p->x - topsite->coord.x;
	   fast = 0;
	   if(((!right_of_site)&(e->b<0.0)) | (right_of_site&(e->b>=0.0)) )
	   {	
         above = dyp>= e->b*dxp;	
		   fast = above;
	   }
	   else 
	   {	
         above = p->x + p->y*e->b > e-> c;
		   if(e->b<0.0) above = !above;
		   if(!above) fast = 1;
	   }
	   if(!fast)
	   {	
         dxs = topsite->coord.x - (e->reg[0])->coord.x;
		   above = e->b * (dxp*dxp - dyp*dyp) <
		           dxs*dyp*(1.0+2.0*dxp/dxs + e->b*e->b);
		   if(e->b<0.0) above = !above;
	   }
   }
   else  /*e->b==1.0 */
   {	
      yl = e->c - e->a*p->x;
	   t1 = p->y - yl;
	   t2 = p->x - topsite->coord.x;
	   t3 = yl - topsite->coord.y;
	   above = t1*t1 > t2*t2 + t3*t3;
   }
   return (el->ELpm==LE ? above : !above);
}


struct Halfedge* EdgeList::ELleftbnd(struct VonoroiPoint *p, double xmin)
{
   int i, bucket;
   struct Halfedge *he;

   /* Use hash table to get close to desired halfedge */
	bucket = (int)((p->x - xmin)/Parent->MySites->deltax * ELhashsize);
	if(bucket<0)
      bucket =0;
	else if(bucket>=ELhashsize)
      bucket = ELhashsize - 1;
	he = ELgethash(bucket);
	if(he == (struct Halfedge *) NULL)
	{   
      for(i=1; 1 ; i++)
	   { 
         if ((he=ELgethash(bucket-i)) != (struct Halfedge *) NULL) break;
		   if ((he=ELgethash(bucket+i)) != (struct Halfedge *) NULL) break;
	   }
	   totalsearch += i;
	}
	ntry++;

   /* Now search linear list of halfedges for the corect one */
	if(he==ELleftend || (he != ELrightend && right_of(he,p)))
	{  
      do 
      {
         he = he -> ELright;
      } while(he!=ELrightend && right_of(he,p));
	   he = he -> ELleft;
	}
	else 
	   do 
      {
         he = he -> ELleft;
      } while (he!=ELleftend && !right_of(he,p));

   /* Update hash table and reference counts */
	if(bucket > 0 && bucket <ELhashsize-1)
	{	
      if(ELhash[bucket] != (struct Halfedge *) NULL) 
			ELhash[bucket]->ELrefcnt--;
		ELhash[bucket] = he;
		ELhash[bucket]->ELrefcnt++;
	}
	return (he);
}

	
/* This delete routine can't reclaim node, since pointers from hash
   table may be present.   */
void EdgeList::ELdelete(struct Halfedge *he)
{
	(he -> ELleft) -> ELright = he -> ELright;
	(he -> ELright) -> ELleft = he -> ELleft;
	he -> ELedge = (struct Edge *)DELETED;
}


struct Halfedge* EdgeList::ELright(struct Halfedge *he)
{
	return (he->ELright);
}

struct Halfedge* EdgeList::ELleft(struct Halfedge *he)
{
	return (he->ELleft);
}


struct Site* EdgeList::leftreg(struct Halfedge *he)
{
	if(he -> ELedge == (struct Edge *)NULL)
      return(Parent->MySites->bottomsite);
	return( he -> ELpm == LE ? 
		he -> ELedge -> reg[LE] : he -> ELedge -> reg[RE]);
}

struct Site* EdgeList::rightreg(struct Halfedge *he)
{
	if(he->ELedge == (struct Edge *)NULL)
      return(Parent->MySites->bottomsite);
	return( he->ELpm == LE ? 
		he->ELedge->reg[RE] : he->ELedge->reg[LE]);
}


Output::Output(CVoronoi* parent, CDC* passDC)
{
   Parent=parent;
   pDC=passDC;
}

int Output::clip_line(struct Edge *e)
{
   //set range to DC clip
   RECT MyRect;
   pDC->GetClipBox(&MyRect);
   pxmin=MyRect.left;
   pxmax=MyRect.right;
   pymin=MyRect.top;
   pymax=MyRect.bottom;
   
   
   struct Site *s1, *s2;
   double x1,x2,y1,y2;

	if(e -> a == 1.0 && e ->b >= 0.0)
	{	s1 = e -> ep[1];
		s2 = e -> ep[0];
	}
	else 
	{	s1 = e -> ep[0];
		s2 = e -> ep[1];
	}

	if(e -> a == 1.0)
	{
		y1 = pymin;
		if (s1!=(struct Site *)NULL && s1->coord.y > pymin)
			 y1 = s1->coord.y;
		if(y1>pymax) return(0);
		x1 = e -> c - e -> b * y1;

      y2 = pymax;
		if (s2!=(struct Site *)NULL && s2->coord.y < pymax) 
			y2 = s2->coord.y;
		if(y2<pymin) return(0);
		x2 = e -> c - e -> b * y2;

      if (((x1>pxmax)&(x2>pxmax))|((x1<pxmin)&(x2<pxmin))) return(0);
		if(x1> pxmax)
		{	x1 = pxmax; y1 = (e -> c - x1)/e -> b;};
		if(x1<pxmin)
		{	x1 = pxmin; y1 = (e -> c - x1)/e -> b;};
		if(x2>pxmax)
		{	x2 = pxmax; y2 = (e -> c - x2)/e -> b;};
		if(x2<pxmin)
		{	x2 = pxmin; y2 = (e -> c - x2)/e -> b;};
	}
	else
	{
		x1 = pxmin;
		if (s1!=(struct Site *)NULL && s1->coord.x > pxmin) 
			x1 = s1->coord.x;
		if(x1>pxmax) return(0);
		y1 = e -> c - e -> a * x1;

      x2 = pxmax;
		if (s2!=(struct Site *)NULL && s2->coord.x < pxmax) 
			x2 = s2->coord.x;
		if(x2<pxmin) return(0);
		y2 = e -> c - e -> a * x2;

      if (((y1>pymax)&(y2>pymax))|((y1<pymin)&(y2<pymin))) return(0);
		if(y1> pymax)
		{	y1 = pymax; x1 = (e -> c - y1)/e -> a;};
		if(y1<pymin)
		{	y1 = pymin; x1 = (e -> c - y1)/e -> a;};
		if(y2>pymax)
		{	y2 = pymax; x2 = (e -> c - y2)/e -> a;};
		if(y2<pymin)
		{	y2 = pymin; x2 = (e -> c - y2)/e -> a;};
	}
	
	//line(x1,y1,x2,y2);
   //out_line();
   CPen aPen;
   COLORREF aColor = 0x00ff0000;         // Initialize with element color
   aPen.CreatePen(PS_SOLID, 1, aColor);

   CPen* pOldPen = pDC->SelectObject(&aPen);  // Select the pen
   pDC->MoveTo((int)x1, (int)y1);
   pDC->LineTo((int)x2, (int)y2);
   pDC->SelectObject(pOldPen);                // Restore the old pen
   return(0);
}


void Output::out_bisector(struct Edge *e)
{
   if(Parent->triangulate && pDC)
   {
      CPen aPen;
      COLORREF aColor = 0x000000ff;         // Initialize with element color
	  aPen.CreatePen(PS_SOLID, 1, aColor);

      CPen* pOldPen = pDC->SelectObject(&aPen);  // Select the pen
      pDC->MoveTo((int)e->reg[0]->coord.x, (int)e->reg[0]->coord.y);
      pDC->LineTo((int)e->reg[1]->coord.x, (int)e->reg[1]->coord.y);
      pDC->SelectObject(pOldPen);                // Restore the old pen
   }
   if(Parent->debug)
   {
	   printf("line(%d) %gx+%gy=%g, bisecting %d %d\n", e->edgenbr,
	         e->a, e->b, e->c, e->reg[LE]->sitenbr, e->reg[RE]->sitenbr);
      ListLines.AddTail(*e);
   }

}


void Output::out_ep(struct Edge *e)
{
   if(Parent->plot && pDC) 
	   clip_line(e);
   int left=e->ep[LE] != (struct Site *)NULL ? e->ep[LE]->sitenbr : -1;
   int right=e->ep[RE] != (struct Site *)NULL ? e->ep[RE]->sitenbr : -1;
   printf("e %d %d %d\n", e->edgenbr, left, right);
   ListEdges.AddTail(*e);
}


void Output::out_vertex(struct Site *v)
{
   printf("vertex(%d) at %f %f\n", v->sitenbr, v->coord.x, v->coord.y);
   ListVertices.AddTail(*v);
}


void Output::out_site(struct Site *s)
{
   ListSites.AddTail(*s);
/*   if(!Parent->triangulate & Parent->plot & !Parent->debug)
	   //circle (s->coord.x, s->coord.y, cradius);
      out_circle ();
   printf("site (%d) at %f %f\n", s->sitenbr, s->coord.x, s->coord.y);

   CFont aFont;
   aFont.CreatePointFont(100, "");
   CFont* pOldFont = pDC->SelectObject(&aFont);
   COLORREF Color(0);          // Initialize with element color
   // Set the text color and output the text
   pDC->SetTextColor(Color);

   int x=(int)s->coord.x;
   int y=(int)s->coord.y;
   char temp[100];
   sprintf(temp, "S(%d)(%d,%d)", s->sitenbr, x, y);
   COLORREF Color(0x00aaaaaa);          // Initialize with element color
   pDC->SetTextColor(Color);
   pDC->TextOut(x, y, temp);
   POINT MyPoint;
   MyPoint.x=x;
   MyPoint.y=y;
   pDC->LPtoDP(&MyPoint);
   x=MyPoint.x;
   y=MyPoint.y;
   pDC->Ellipse(x-2, y-2, x+2, y+2);
   //pDC->SelectObject(pOldFont);

  */
}


void Output::out_triple(struct Site *s1, struct Site *s2, struct Site *s3)
{
   printf("circle through left=%d right=%d bottom=%d\n", 
		      s1->sitenbr, s2->sitenbr, s3->sitenbr);
}


Geometry::Geometry(CVoronoi* parent)
{
   Parent=parent;
	Parent->MyMemory->freeinit(&efl, sizeof(struct Edge));
	nvertices = 0;
	nedges = 0;
}

Geometry::~Geometry()
{
	//Parent->MyMemory->myfree(&efl);
 
}



struct Edge* Geometry::bisect(struct	Site *s1, struct Site *s2)
{
   double dx,dy,adx,ady;
   struct Edge *newedge;

	newedge=(struct Edge *) Parent->MyMemory->getfree(&efl);
	newedge->reg[0] = s1;
	newedge->reg[1] = s2;
	ref(s1); 
	ref(s2);
	newedge->ep[0] = (struct Site *) NULL;
	newedge->ep[1] = (struct Site *) NULL;

	dx = s2->coord.x - s1->coord.x;
	dy = s2->coord.y - s1->coord.y;
	adx = dx>0 ? dx : -dx;
	ady = dy>0 ? dy : -dy;
	newedge->c = (double)(s1->coord.x * dx 
                        + s1->coord.y * dy + (dx*dx + dy*dy)*0.5);
	if(adx>ady)
	{	
      newedge -> a = 1.0; 
      newedge -> b = dy/dx; 
      newedge -> c /= dx;
   }
	else
	{	
      newedge -> b = 1.0; 
      newedge -> a = dx/dy; 
      newedge -> c /= dy;
   }

	newedge -> edgenbr = nedges;
	Parent->MyOutput->out_bisector(newedge);
	nedges += 1;
	return(newedge);
}


struct Site* Geometry::intersect(struct Halfedge *el1, struct Halfedge *el2)
{
   struct	Edge *e1,*e2, *e;
   struct  Halfedge *el;
   double d, xint, yint;
   int right_of_site;
   struct Site *v;

	e1 = el1 -> ELedge;
	e2 = el2 -> ELedge;
	if(e1 == (struct Edge*)NULL || e2 == (struct Edge*)NULL) 
		return ((struct Site *) NULL);
	if (e1->reg[1] == e2->reg[1]) return ((struct Site *) NULL);

	d = e1->a * e2->b - e1->b * e2->a;
	if (-1.0e-10<d && d<1.0e-10) return ((struct Site *) NULL);

	xint = (e1->c*e2->b - e2->c*e1->b)/d;
	yint = (e2->c*e1->a - e1->c*e2->a)/d;

	if( (e1->reg[1]->coord.y < e2->reg[1]->coord.y) ||
	    (e1->reg[1]->coord.y == e2->reg[1]->coord.y &&
		e1->reg[1]->coord.x < e2->reg[1]->coord.x) )
	{	el = el1; e = e1;}
	else
	{	el = el2; e = e2;};
	right_of_site = xint >= e -> reg[1] -> coord.x;
	if ((right_of_site && el -> ELpm == LE) ||
	   (!right_of_site && el -> ELpm == RE)) return ((struct Site *) NULL);

	v = (struct Site *) Parent->MyMemory->getfree(&(Parent->MySites->sfl));
	v -> refcnt = 0;
	v -> coord.x = xint;
	v -> coord.y = yint;
	return(v);
}

void Geometry::ref(struct Site *v)
{
   v->refcnt++;
}


void Geometry::deref(struct	Site *v)
{
   v->refcnt--;
   if(v->refcnt == 0) Parent->MyMemory->makefree((struct Freenode*)v, &(Parent->MySites->sfl));
}


void Geometry::endpoint(struct Edge *e, int	lr, struct Site *s)
{
   e -> ep[lr] = s;
   ref(s);
   if(e -> ep[RE-lr]== (struct Site *) NULL) return;
   Parent->MyOutput->out_ep(e);
   deref(e->reg[LE]);
   deref(e->reg[RE]);
   Parent->MyMemory->makefree((struct Freenode*)e, &efl);
}


double Geometry::dist(struct Site *s, struct Site *t)
{
   double dx,dy;
	dx = s->coord.x - t->coord.x;
	dy = s->coord.y - t->coord.y;
	return((double)sqrt(dx*dx + dy*dy));
}


void Geometry::makevertex(struct Site *v)
{
   v -> sitenbr = nvertices;
   nvertices++;
   Parent->MyOutput->out_vertex(v);
}

Memory::Memory(CVoronoi* parent)
{
   Parent=parent;
   total_alloc=0;
}


void Memory::freeinit(struct Freelist* fl, int size)
{
   fl -> head = (struct Freenode *) NULL;
   fl -> nodesize = size;
}


// add curr to fl's head
void Memory::makefree(struct Freenode *curr, struct Freelist *fl)
{
   curr -> nextfree = fl -> head;
   fl -> head = curr;
}


// get a Freenode from fl(from fl's head), 
char* Memory::getfree(struct	Freelist *fl)
{
   struct Freenode *fn;
   // if there is nothing left in fl, allocate them.
   if(fl->head == (struct Freenode *) NULL)
   {  
      int sqrt_nsites=Parent->MySites->sqrt_nsites;
      int NodeSize=fl->nodesize;
      int AllocSize=sqrt_nsites * NodeSize;
      if (!(fn=(Freenode*)malloc(AllocSize)))
      {    fprintf(stderr,"Insufficient memory processing site %d (%d bytes in use)\n",
	   	   Parent->MySites->siteidx, total_alloc);
           exit(1);
      }
      total_alloc+=AllocSize;
      char* MemEnd=(char*)fn+AllocSize;
	   for(char* i=(char*)fn; i<MemEnd; i+=NodeSize) 	
		   makefree((struct Freenode *)i, fl);
   }
   fn = fl -> head;
   fl -> head = fn -> nextfree;
   return((char *)fn);
}


char* Memory::myalloc(unsigned int n)
{
   char *t;
   if ((t=(char*)malloc(n)) == (char *) 0)
   {    fprintf(stderr,"Insufficient memory processing site %d (%d bytes in use)\n",
	   	Parent->MySites->siteidx, total_alloc);
        exit(1);
   }
   total_alloc+= n;
   return(t);
}

void Memory::myfree(struct Freelist* fl)
{
   free(fl);
}