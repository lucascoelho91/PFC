/*
 * graphBuilder.h
 *
 *  Created on: Apr 28, 2014
 *      Author: lucas
 */

#ifndef GRAPHBUILDER_H_
#define GRAPHBUILDER_H_

#include <iostream>
#include <vector>
#include <queue>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>

#include <voronoi/Vector2.h>
#include <voronoi/node.h>
#include <voronoi/Robot.h>
#include <voronoi/rgb.h>

#include <ros/ros.h>

using namespace std;

class Graph  //
{
    public:
        int squareSize;        //tamanho do quadrado que um vértice ocupa, EM PIXELS
        double sizeMetersPixel;

        Vector2 dim;
        Vector2 sizeMap; //tamanho em metros do mapa

        Vector2 vertices;

        
        rgb** colors;        //colors é o mapa da região da forma que ele foi copiado do arquivo de entrada
        rgb** visualization; //essa variável é uma cópia de colors e serve para ser editada para incluir os pixels para visualização da região
 

    private:
		std::vector < std::vector <node*> > matrixGraph; //matriz de vértices que é o grafo
       rgb** image;



        int8_t drawLines; //especifica śe é para desenhar linhas na visualização do grafo montado

        FILE* mapFile;
        FILE* outmap;

    public:

        void GetParametersAndBuildGraph(); // runs this shit and everything together
        void BuildGraph(int threshold);        //monta o grafo, lendo o mapa de entrada, inicializando os vértices e estabelecendo os vizinhos

        void ClearGraph(); //limpa o grafo, zerando os valores de owner, visited, have_robot, entre outros

        node* PoseToNode(double y, double x);          //dada uma posição, o programa responde o vértice que esse ponto está contido

        Graph ();

        void openMapFile(char* file, char* outFile);
        void printFile();
        void readMap(uint8_t threshold);
        void verticesAllocation();
        void connectNeighbors();

        double getSquareSize();
        double getSizeMetersPixel();
        void FillSquare(double x, double y, rgb color);

        void DrawSquare(int size, int i, int j, rgb pixel);
        void DrawArrow(int size, int width, int i, int j, rgb pixel);

        void init (double sizeDiscretization, char* file, double sizeMetersPixel, char* outFile);
        node* getNodeByIndex(int x, int y);
};


#endif /* GRAPHBUILDER_H_ */
