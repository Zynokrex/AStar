//Biel González Garriga 1551813
# include <stdio.h>
# include <stdlib.h>
# include <string.h>
# include <math.h>
# include <float.h>
# include <limits.h>

# define UMAX 432444444
# define MAXARST 15

//Weighted arrow
typedef struct{
    char carrer[12];
    unsigned numnode;   // Posició al vector del node al que arribem
    double llargada;    // Pes de l'aresta
}infoaresta;

//Graph vertex
typedef struct{
    long int id;
    double latitud,longitud;
    int narst;                      // Nombre d'arestes del node
    infoaresta arestes[MAXARST];    /*Vector que conté la informació
                                     *de les arestes del node*/
}node;

typedef struct{
    double g;
    unsigned parent;
} AStarPath;

typedef struct QueueElementstruct {
    unsigned v; //Index del node al vector que els emagatzema
    struct QueueElementstruct *seg;
} QueueElement;

typedef QueueElement * PriorityQueue;

typedef struct {
    double f;
    unsigned IsOpen;
} AStarControlData;

// Funció heuristica
double distancia(node, node);

// Funció de busqueda a llista
unsigned BuscaPunt(long int, node*, int);

// Funcions de cua de prioritat
unsigned ExtractMin(PriorityQueue *);
void RequeueWithPriority(unsigned, PriorityQueue *, AStarControlData *);
unsigned AddWithPriority(unsigned, PriorityQueue *, AStarControlData *);
int IsEmpty( PriorityQueue Q ){ return ( Q == NULL ); }

//Funció AEstrella principal
unsigned AStar(node *, AStarPath *, unsigned, unsigned, unsigned);

void ExitError(const char *miss, int errcode) {
    fprintf (stderr, "\nERROR: %s.\nStopping...\n\n", miss); exit(errcode);
}


int main(int argc, char *argv[]){
    //Lectura d'arguments
    if(argc!=3){
        fprintf(stderr, "%s startingNode endingNode\n",argv[0]);
        return -1;
    }

    //Lectura d'arxius
    FILE *dades;
    node *punts;
    int c,nnodes=0,i;

    dades=fopen("Nodes.csv","r");
    if(dades==NULL)
    {
        printf("\nMap file couldn't be accessed\n");
        return 1;
    }
    while((c=fgetc(dades))!=EOF)
    {
        if(c=='\n') nnodes++;
    }
    rewind(dades);

    if((punts=(node *)malloc(nnodes*sizeof(node)))==NULL)
    {
        printf("Error when allocating memory for the node vector\n");
        return 2;
    }
    for(i=0;i<nnodes;i++){
        fscanf(dades,"%ld;",&(punts[i].id));
        fscanf(dades,"%lf;",&(punts[i].latitud));
        fscanf(dades,"%lf\n",&(punts[i].longitud));
    }
    fclose(dades);

    FILE *dadescarrer;
    dadescarrer=fopen("Carrers.csv","r");
    if(dadescarrer==NULL)
    {
        printf("\nThe file containing the streets couldn't be accessed\n");
        return 1;
    }
    char idcarrer[12];
    long int idnode,idnodeanterior;
    unsigned posnode,posnodeanterior, ncarrers=0;
    double dist;

    while((c=fgetc(dadescarrer))!=EOF)
    {
        if(c=='\n') ncarrers++;
    }
    rewind(dadescarrer);

    for(i=0; i<ncarrers; i++){
        fscanf(dadescarrer,"id=%[0-9];",idcarrer);
        fscanf(dadescarrer,"%ld",&(idnodeanterior));
        posnodeanterior=BuscaPunt(idnodeanterior,punts,nnodes);

        while(fgetc(dadescarrer)!='\n'){
            fscanf(dadescarrer,"%ld",&(idnode));
            posnode=BuscaPunt(idnode,punts,nnodes);

            if((posnodeanterior!=UMAX) && (posnode != UMAX)){
                dist=distancia(punts[posnodeanterior], punts[posnode]);

                punts[posnodeanterior].arestes[punts[posnodeanterior].narst].numnode=posnode;
                strcpy(punts[posnodeanterior].arestes[punts[posnodeanterior].narst].carrer,idcarrer);
                punts[posnodeanterior].arestes[punts[posnodeanterior].narst].llargada=dist;
                punts[posnodeanterior].narst++;

                punts[posnode].arestes[punts[posnode].narst].numnode=posnodeanterior;
                strcpy(punts[posnode].arestes[punts[posnode].narst].carrer,idcarrer);
                punts[posnode].arestes[punts[posnode].narst].llargada=dist;
                punts[posnode].narst++;

                posnodeanterior=posnode;
            }
        }
    }

    //Codi principal
    unsigned idxIni, idxFin;
    AStarPath PathData[nnodes];
    idxIni= BuscaPunt(atol(argv[1]),punts,nnodes);
    idxFin=BuscaPunt(atol(argv[2]),punts,nnodes);
    if(idxIni==UMAX){
        printf("The initial node doesen't exist\n");
        return -1;
    }
    if(idxFin==UMAX){
        printf("The final node doesen't exist\n");
        return -1;
    }

    unsigned r = AStar(punts, PathData, nnodes, idxIni, idxFin);
    if(r == -1) ExitError("in allocating memory for the OPEN list in AStar", 21);
    else if(!r) ExitError("no solution found in AStar", 7);
    register unsigned v=idxFin, pv=PathData[v].parent, ppv; PathData[idxFin].parent=UINT_MAX;
    while(v != idxIni) { ppv=PathData[pv].parent; PathData[pv].parent=v; v=pv; pv=ppv; }
    printf("# La distancia de %ld a %ld es de %lf metres.\n",punts[idxIni].id, punts[idxFin].id, PathData[idxFin].g);
    printf("# Cami optim:\n");
    printf("Id=%010ld  |  %lf  |  %lf  |  Dist = %lf\n", punts[idxIni].id, punts[idxIni].latitud, punts[idxIni].longitud, PathData[idxIni].g);
    for(v=PathData[idxIni].parent ; v !=UINT_MAX ; v=PathData[v].parent) {
        printf("Id=%010ld  |  %lf  |  %lf  |  Dist = %lf\n", punts[v].id, punts[v].latitud, punts[v].longitud, PathData[v].g);
    }
    printf("# ---------------------------------\n");

    return 0;
}

unsigned BuscaPunt(long int ident, node l[], int n){
    unsigned U=n-1, M=U/2, P=0;
    while(U > M && M > P){
        if(l[M].id > ident){
            U = M;
            M = (U-P)/2 + P;
        }
        else if(l[M].id < ident) {
            P = M;
            M = (U-P)/2 + P;
        }
        else return M;	//trobat
    }
    if(l[P].id == ident) return P;	//a un extrem
    else if(l[U].id == ident) return U;	//o a l'altre

    return UMAX;	//no trobat
}

double distancia(node ini, node fin){
    double dist, xini, yini, zini, xfin, yfin, zfin;
    double R=6371000;
    double conv=M_PI/180;

    xini=R*cos(ini.longitud*conv)*cos(ini.latitud*conv);
    xfin=R*cos(fin.longitud*conv)*cos(fin.latitud*conv);

    yini=R*sin(ini.longitud*conv)*cos(ini.latitud*conv);
    yfin=R*sin(fin.longitud*conv)*cos(fin.latitud*conv);

    zini=R*sin(ini.latitud*conv);
    zfin=R*sin(fin.latitud*conv);

    dist=sqrt(pow(xfin-xini,2)+pow(yfin-yini,2)+pow(zfin-zini,2));

    return dist;
}
unsigned AStar(node *Graph, AStarPath *PathData, unsigned GrOrder, unsigned node_start, unsigned node_goal){
    register unsigned i;
    double g_curr_node_succ;
    PriorityQueue Open = NULL;
    AStarControlData *Q;

    if((Q = (AStarControlData *) malloc(GrOrder*sizeof(AStarControlData))) == NULL)
        ExitError("when allocating memory for the AStar Control Data vector", 73);

    for(i=0; i < GrOrder; i++){
        PathData[i].g = DBL_MAX;
        Q[i].IsOpen = 0;
    }

    PathData[node_start].g = 0.0;
    PathData[node_start].parent = UINT_MAX;
    Q[node_start].f = distancia(Graph[node_start], Graph[node_goal]);

    if(!AddWithPriority(node_start, &Open, Q)) return -1;

    while(!IsEmpty(Open)){
        unsigned node_curr;

        if((node_curr = ExtractMin(&Open)) == node_goal){
            free(Q);
            return 1;
        }

        for(i=0; i < Graph[node_curr].narst ; i++){

            unsigned node_succ = Graph[node_curr].arestes[i].numnode;
            g_curr_node_succ = PathData[node_curr].g + Graph[node_curr].arestes[i].llargada;

            if( g_curr_node_succ < PathData[node_succ].g ){
                PathData[node_succ].parent = node_curr;

                if(PathData[node_succ].g == DBL_MAX){
                    Q[node_succ].f = g_curr_node_succ + distancia(Graph[node_succ], Graph[node_goal]);
                }
                else{
                    Q[node_succ].f = g_curr_node_succ +(Q[node_succ].f-PathData[node_succ].g);
                }
                PathData[node_succ].g = g_curr_node_succ;

                if(!Q[node_succ].IsOpen) {
                    if(!AddWithPriority(node_succ, &Open, Q)) return -1;
                }
                else RequeueWithPriority(node_succ, &Open, Q);
            }
        }
        Q[node_curr].IsOpen = 0;
    } /* Main loop while */
    return 0;
}
unsigned ExtractMin(PriorityQueue *Pq){
    PriorityQueue first = *Pq;
    unsigned v = first->v;

    *Pq = (*Pq)->seg;
    free(first);

    return v;
}

void RequeueWithPriority(unsigned v, PriorityQueue *Pq, AStarControlData * Q){
    register QueueElement * prepv;
    if((*Pq)->v == v) return;

    for(prepv = *Pq; prepv->seg->v != v; prepv = prepv->seg);
    QueueElement * pv = prepv->seg;
    prepv->seg = pv->seg;
    free(pv);

    AddWithPriority(v, Pq, Q);
}

unsigned AddWithPriority(unsigned v, PriorityQueue *Pq, AStarControlData * Q) {
    register QueueElement *q;
    QueueElement *aux = (QueueElement *) malloc(sizeof(QueueElement));
    if (aux == NULL) return 0;

    aux->v = v;
    double costv = Q[v].f;
    Q[v].IsOpen = 1;

    if (*Pq == NULL || !(costv > Q[(*Pq)->v].f)) {
        aux->seg = *Pq;
        *Pq = aux;
        return 1;
    }

    for (q = *Pq; q->seg && Q[q->seg->v].f < costv; q = q->seg);
    aux->seg = q->seg;
    q->seg = aux;
    return 1;
}
