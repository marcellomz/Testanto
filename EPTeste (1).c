#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <malloc.h>
#include <float.h>
//#include<string.h>
#include<limits.h>
//!Para representar um valor double infinito, use DBL_MAX

//!                                                                   OS VERTICES DO GRAFO SAO INDEXADOS A PARTIR DE 1


#define VERTICE_INVALIDO NULL  /* numero de vertice invalido ou ausente */
#define AN -1                /* aresta nula */


typedef int TipoPeso;

typedef struct taresta {
  int vdest;
  TipoPeso peso;
  struct taresta * prox;
} TipoAresta;

typedef TipoAresta *TipoApontador;

typedef struct {
  TipoApontador *listaAdj;
  int numVertices;
  int numArestas;
} TipoGrafo;

int partida;
int nVertices;
int nArestas;
int vertice;
int aresta;
int peso;


/*!
  bool inicializaGrafo(TipoGrafo* grafo, int nv): Inicializa um grafo com nv vertices.
  Vertices vao de 0 a nv-1.
  Aloca o vetor listaAdj e o inicializa com null.                                                                       INCOMPLETO
  Retorna true se inicializou com sucesso e false caso contrario
*/
bool inicializaGrafo(TipoGrafo* grafo, int nv) {
    //int vertices = g.numVertices;
    //int arestas = g.numArestas;
    int i;
    for (i = 1; i <= nv; i++) {
        grafo[i].listaAdj = NULL;
    }
}

/*! int obtemNrVertices(TipoGrafo* grafo): retorna o numero de vertices do grafo */
int obtemNrVertices(TipoGrafo* grafo) {
    return nVertices;
}

/*! int obtemNrArestas(TipoGrafo* grafo): retorna o numero de arestas do grafo */
int obtemNrArestas(TipoGrafo* grafo) {
    return nArestas;
}

/*!
  bool verificaValidadeVertice(int v, TipoGrafo *grafo): verifica se o nr do vertice eh valido no grafo,
  ou seja, se ele é maior que zero e menor ou igual ao nr total de vertices do grafo.
*/
bool verificaValidadeVertice(int v, TipoGrafo *grafo) {
    if (v > 0 && v < nVertices)
        return true;
}

/*!
  bool existeAresta(int v1, int v2, TipoGrafo *grafo):
  Retorna true se existe a aresta (v1, v2) no grafo e false caso contrário
*/
bool existeAresta(int v1, int v2, TipoGrafo *grafo){
    TipoApontador p = grafo[v1].listaAdj;
    while(p) {
        if(p->vdest == v2) {
            return true;
        }
        p = p->prox;
    }
    return false;
}

/*!
  void insereAresta(int v1, int v2, TipoPeso peso, TipoGrafo *grafo):
  Insere a aresta (v1, v2) com peso "peso" no grafo.
  Nao verifica se a aresta ja existia.
*/
void insereAresta(int v1, int v2, TipoPeso peso, TipoGrafo *grafo){
    if(existeAresta(v1, v2, grafo));
    TipoAresta* nova = (TipoAresta*)malloc(sizeof(TipoAresta));
    nova->vdest = v2;
    nova->peso = peso;
    nova->prox = grafo[v1].listaAdj;
    grafo[v1].listaAdj = nova;
}

/*!
  TipoPeso obtemPesoAresta(int v1, int v2, TipoGrafo *grafo):
  Retorna o peso da aresta (v1, v2) no grafo se ela existir e AN caso contrário
*/
TipoPeso obtemPesoAresta(int v1, int v2, TipoGrafo *grafo) {
    TipoApontador p = grafo[v1].listaAdj;
    while(p) {
        if(p->vdest == v2) {
            return p->peso;
        }
        else {
            return AN;
        }
        p = p->prox;
    }
}


/*!
  bool removeArestaObtendoPeso(int v1, int v2, TipoPeso* peso, TipoGrafo *grafo);
  Remove a aresta (v1, v2) do grafo colocando AN em sua celula (representando ausencia de aresta).                          INCOMPLETO
  Se a aresta existia, coloca o peso dessa aresta em "peso" e retorna true,
  caso contrario retorna false (e "peso" é inalterado).
*/
bool removeArestaObtendoPeso(int v1, int v2, TipoPeso* peso, TipoGrafo *grafo) {

}


/*!
   bool listaAdjVazia(int v, TipoGrafo* grafo):
   Retorna true se a lista de adjacencia (de vertices adjacentes) do vertice v é vazia, e false caso contrário.
*/
bool listaAdjVazia(int v, TipoGrafo* grafo) {
    TipoApontador p = grafo[v].listaAdj;
    while(p) {
        if(p->vdest) {
            return false;
        }
        p = p->prox;
    }
    return true;
}

/*!
   TipoApontador primeiroListaAdj(int v, TipoGrafo* grafo):                                                                  INCOMPLETO
   Retorna o primeiro vertice da lista de adjacencia de v
   ou VERTICE_INVALIDO se a lista de adjacencia estiver vazia.
*/
TipoApontador primeiroListaAdj(int v, TipoGrafo* grafo) {

}

/*!
   TipoApontador proxListaAdj(int v, TipoGrafo* grafo, TipoApontador atual):
   Trata-se de um iterador sobre a lista de adjacência do vertice v.
   Retorna o proximo vertice adjacente a v, partindo do vertice "atual" adjacente a v                                        INCOMPLETO
   ou VERTICE_INVALIDO se a lista de adjacencia tiver terminado sem um novo proximo.
*/
TipoApontador proxListaAdj(int v, TipoGrafo* grafo, TipoApontador atual) {

}

/*!
    void imprimeGrafo(TipoGrafo* grafo):
    Imprime a matriz de adjacencia do grafo.
    Assuma que cada vértice e cada peso de aresta são inteiros de até 2 dígitos.
*/
void imprimeGrafo(TipoGrafo* grafo) {
    int nv = obtemNrVertices(grafo);
    for (int i = 1; i <= nv; i++) {
        TipoAresta* p = grafo[i].listaAdj;
        while(p) {
            printf("v%d:  %d ", i, p->vdest);
            printf("\n");
            p = p->prox;
        }
    }
}

/*! Nao precisa fazer nada para matrizes de adjacencia */
void liberaGrafo(TipoGrafo* grafo) {


}

/*! int verticeDestino(TipoApontador p):
   retorna o vertice destino da aresta apontada por p */
int verticeDestino(TipoApontador p, TipoGrafo* grafo) {


}

//!/////////////////////////////////////////////////////////////////////HEAP/////////////////////////////////////////////////////////////////////

struct MinHeap {
    int *pesos;
    int tamanho;
    int indiceNo;
};

struct MinHeap mh;

void iniciarMinHeap (int tamanho) {
    mh.pesos = (int *) malloc (sizeof (int) * tamanho);
    int i;
    for (i = 1; i <= tamanho; i++)
        mh.pesos[i] = DBL_MAX;

    mh.tamanho = tamanho;
    mh.indiceNo = 0;
}


/* get left child index */
int esquerda (int index) {
    return 2 * index;
}

/* get right child index */
int direita (int index) {
    return 2 * index + 1;
}

/* get parent index */
int pai (int index) {
    return (index / 2);
}


void heapyfy_push (int index) {
    if (index >= mh.tamanho)
        return;
    int parentNodeIndex = pai(index);
    int tmp;
    if (index != 1) {
        if(mh.pesos[parentNodeIndex] > mh.pesos[index]) {
            tmp = mh.pesos[parentNodeIndex];
            mh.pesos[parentNodeIndex] = mh.pesos[index];
            mh.pesos[index] = tmp;
            heapyfy_push (parentNodeIndex);
        }
    }
}

void push (int valor) {
    mh.indiceNo++;
    // check if heap is full or not full
    if (mh.indiceNo < mh.tamanho) {
        mh.pesos[mh.indiceNo] = valor;
        heapyfy_push (mh.indiceNo);
    }
    else {
        printf ("Heap is full\n");
        printf ("Not possible to push %d\n", valor);
        exit (-1);
    }
}

void heapyfy_pop (int index) {
    int tmp;
    int esq = esquerda(index);
    int dir = direita(index);
    int min_index;
    if (dir >= mh.indiceNo) {
        if (esq >= mh.indiceNo)
            return;
        else
            min_index = esq;
    }
    else {
        if (mh.pesos[esq] <= mh.pesos[dir])
            min_index = esq;
        else
            min_index = dir;
    }

    if (mh.pesos[index] > mh.pesos[min_index]) {
        tmp = mh.pesos[min_index];
        mh.pesos[min_index] = mh.pesos[index];
        mh.pesos[index] = tmp;
        heapyfy_pop (min_index);
    }
}

bool vazio () {
    if (mh.indiceNo == 0)
        return true;
    else
        return false;
}

int pop () {
    int minvalue;
    if (vazio() == false) {
        minvalue = mh.pesos[1];
        mh.pesos[1] = mh.pesos[mh.indiceNo];
        mh.indiceNo--;
        if (mh.indiceNo > 1)
            heapyfy_pop (1);
        return minvalue;
    }

}

void apagarHeap () {
    free (mh.pesos);
}


void printHeap () {
    int i = 1;
    for (i; i <= mh.indiceNo; i++)
        printf ("%d ", mh.pesos[i]);
    printf ("\n");

}




//!//////////////////////////////////////////////////////////////DIJKSTRA E FUNCOES/////////////////////////////////////////////////////////////////////

//relaxamento de (u, v)
void relax (TipoGrafo* grafo, int *d, int *p, int u, int v) {
    TipoApontador ponteiro = grafo[u].listaAdj;
    while(ponteiro && ponteiro->vdest != v) {
        ponteiro = ponteiro->prox;
    }
    if(ponteiro) {
        if(d[v] > d[u] + ponteiro->peso) {
            d[v] = d[u] + ponteiro->peso;
            p[v] = u;
        }
    }
}

bool existeAberto(TipoGrafo *grafo, int *aberto) {
    int i;
    for(i = 1; i <= nVertices; i++) { //varre o arranjo de abertos e retorna se tiver um aberto
        if(aberto[i] == 1) return true;
    }
    return false;
}

int menorDist(TipoGrafo *grafo, int *aberto, int *d) {
    int i;
    for(i = 1; i <= nVertices; i++)
        if(aberto[i] == 1) {
            //printf("Vetor aberto: %d\n", i);
            break; //!sai do laço quando encontra o PRIMEIRO aberto
        }

    if(i > nVertices) return -1; // se varreu e nao encontrou, retorna -1
    int menor = i;
    for (i = menor + 1; i <= nVertices; i++) {
        if(aberto[i] == 1 && d[menor] > d[i]) {
            menor = i;
        }
        return menor;
    }
}

//algoritmo de Dijkstra
int *dijkstra (TipoGrafo* grafo, int s) { //vertice inicial s
    int u;
    int *d = (int*) malloc(nVertices * sizeof(int) + 1); //aloca memoria para array de distancias
    int p[nVertices + 1]; //arranjo de predecessores
    //bool aberto[nVertices + 1]; //arranjo de vetores abertos
    int aberto[nVertices + 1];
    //inicializacao
    int v;
    for (v = 1; v <= nVertices; v++) {
        d[v] = DBL_MAX;
        p[v] = -1;
        //printf("D(v1 -> v%d) = %d\n", v, d[v]);
    }
    d[s] = 0; //distancia ate ela mesma eh zero
    //imprimirDistancias(d);
    int i;
    for(i = 1; i <= nVertices; i++) {
        aberto[i] = 1; //inicializa todos como abertos
        //printf("%d\n", aberto[i]);
    }

    //printf("foda-se");
    iniciarMinHeap(nVertices + 1);
    for(int i = 0; i < nVertices; i++) {
        push(DBL_MAX);
    }
    //printHeap ();





     while(existeAberto(grafo, aberto)) { //enquanto tiver algum vertice aberto
        u = menorDist(grafo, aberto, d); //olha entre todos qual tem a menos distancia ate ele
        aberto[u] = 0; //fechou
        TipoApontador ponteiro = grafo[u].listaAdj;
        while(ponteiro) {
            relax(grafo, d, p, u, ponteiro->vdest);
            ponteiro = ponteiro->prox;
        }
    }
    return(d);

//iniciar fila de prioridades com todos os vertices



/*
    while(!vazio()) {
        u = pop();
        printHeap();
        aberto[u] = 0;
        TipoApontador ponteiro = grafo[u].listaAdj;
        while(ponteiro) {
            relax(grafo, d, p, u, ponteiro->vdest);
            ponteiro = ponteiro->prox;
        }
    }
    return(d); */
}




/*!
    while Q = ∅ do
    Remova v de Q tal que d[v] é mínimo (vértice de maior prioridade);
    foreach adjacente u de v do
    if d[u] > d[v] + pesoAresta(v, u) then
    d[u] = d[v] + pesoAresta(v, u);
    Antecessor[u] = v;
*/

//!////////////////////////////////////////////////////////////VERSAO COM HEAP///////////////////////////////////////////////////////////////
//!////////////////////////////////////////////////////////////VERSAO COM HEAP///////////////////////////////////////////////////////////////
    /*
    while(existeAberto(grafo, aberto)) { //enquanto tiver algum vertice aberto
        u = menorDist(grafo, aberto, d); //olha entre todos qual tem a menos distancia ate ele
        aberto[u] = 0; //fechou
        TipoApontador ponteiro = grafo[u].listaAdj;
        while(ponteiro) {
            relax(grafo, d, p, u, ponteiro->vdest);
            ponteiro = ponteiro->prox;
        }
    }
    return(d);
    */


//auxiliar para acompanhar mudancas no vetor de distancias
void imprimirDistancias(int* d) {
    int v;
    for (v = 1; v <= nVertices; v++) {
        printf("D(v1 -> v%d) = %d\n", v, d[v]);
    }
}

//auxiliar para imprimir vetor de abertos
void imprimirAbertos (int *aberto) {
    int v;
    for (v = 1; v <= nVertices; v++) {
            printf("Vetor de abertos: %d\n", aberto[v]);
    }
}


void carregaArquivo (TipoGrafo *g) {
    FILE *f = fopen("/home/marcello/Dropbox/USP/9 Semestre/AED II/EP 1/testeEntrada.txt", "r");
    if (f == NULL) {
        printf("Erro na abertura do arquivo");
    }
        //primeira linha, vertice de partida
        fscanf(f, "%d", &partida);
        //printf("Vertice de partida: %d\n", partida);
        //segunda linda, numero de vertices e arestas
        fscanf(f, "%d %d", &nVertices, &nArestas);
        //printf("Numero de vertices: %d\n", nVertices);
        //printf("Numero de arestas: %d\n", nArestas);
        //linhas seguintes, insercao de arestas
        //printf("Grafo:\n");
        for(int i = 1; i <= nArestas; i++) {
            fscanf(f, "%d %d %d", &vertice, &aresta, &peso);
            insereAresta(vertice, aresta, peso, g);
        }
        fclose(f);
}

//!///////////////////////////////////////////////////////////////////////MAIN/////////////////////////////////////////////////////////////////////
void main() {

    TipoGrafo *g = (TipoGrafo*)malloc(sizeof(TipoGrafo) * 10);
    carregaArquivo(g);
    imprimeGrafo(g);
    printf("\n");

    int *resp = dijkstra(g, 1);
    for(int i = 1; i <= nVertices; i++) {
        printf("D(v1 -> v%d) = %d\n", i, resp[i]);
    }


/*
//! testes heap
  iniciarMinHeap(6);
  for (int i = 0; i <=5; i++) {
    push(1);
  }
    push (DBL_MAX);
    push (8);
    push (-6);
    push (3);
    push (1);
    printf ("heap after pusshing DBL_MAX 8 -6 3 1 -5\n");
    printHeap ();
    pop ();
    printf ("heap array after pop \n");
    printHeap ();
    push (2);
    printf ("heap array after pushing 2\n");
    printHeap ();*/

}
