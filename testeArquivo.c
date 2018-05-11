#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <malloc.h>
void main () {
    int partida;
    int nVertices;
    int nArestas;
    int vertice;
    int aresta;
    int peso;
    FILE *f = fopen("/home/marcello/Dropbox/USP/9 Semestre/AED II/EP 1/testeEntrada.txt", "r");
    if (f == NULL) {
        printf("Erro na abertura do arquivo");
    }
        fscanf(f, "%d", &partida);
        printf("%d\n", partida);
        fscanf(f, "%d %d", &nVertices, &nArestas);
        printf("%d ", nVertices);
        printf("%d\n", nArestas);
        for(int i = 1; i <=3; i++) {
            fscanf(f, "%d %d %d", &vertice, &aresta, &peso);
            printf("%d %d %d\n", vertice, aresta, peso);
        }
        fclose(f);
}
