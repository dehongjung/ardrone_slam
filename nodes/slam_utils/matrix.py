#!/usr/bin/env python

'''
	Universidade de Brasilia 2016
    Laboratorio de Automacao e Robotica
    Autores: De Hong Jung
             Rodrigo Carvalho
    Programa: Biblioteca com funcoes de tratamento de matrizes
'''

import numpy as np 

## Funcao que insere uma matriz 3x3 em outra matriz 
## In: Matriz original, Matriz a ser inserida
## Out: Matriz final
def insertMatrix (M_old, M_new):
	size = len(M_old)
	M = np.insert(M_old, size, np.array([[0], [0], [0]]), 0)
	M = np.insert(M, size, np.array([[0], [0], [0]]), 1)

	ii = 0
	jj = 0
	for i in range(len(M)-3, len(M)):
		for j in range(len(M)-3, len(M)):
			M[i,j] = M_new[ii, jj]
			jj += 1
		ii += 1
		jj = 0

	return M

## Funcao que insere uma matriz 3x3 em outra matriz 
## In: Matriz original, Matriz a ser inserida
## Out: Matriz final
def insertMatrixRight (M_old, pos):
	M = np.insert(M_old, pos, np.array([[0], [0], [0]]), 1)

	return M

def removeMatrixRight (M_old, pos):
	M = np.delete(M_old, (pos, pos+1, pos+2) , axis = 1)

	return M	

## Funcao que remove uma matriz 3x3 de uma outra matriz
## In: Matriz original, index da matriz a ser removida
## Out: Matriz final
def removeMatrix (M_old, index):
	index = index - 1
	index = 6 + index*3
	M = np.delete(M_old, (index, index+1, index+2), axis=0)
	M = np.delete(M, (index, index+1, index+2), axis=1)

	return M

def selectMatrix (M, index):
	index = index - 1
	index = 6 + index*3
	M_selected = np.array([
							[M[0][0], M[0][1], M[0][2], M[0][3], M[0][4], M[0][5], 0.0, 0.0, 0.0],
							[M[1][0], M[1][1], M[1][2], M[1][3], M[1][4], M[1][5], 0.0, 0.0, 0.0],
							[M[2][0], M[2][1], M[2][2], M[2][3], M[2][4], M[2][5], 0.0, 0.0, 0.0],
							[M[3][0], M[3][1], M[3][2], M[3][3], M[3][4], M[3][5], 0.0, 0.0, 0.0],
							[M[4][0], M[4][1], M[4][2], M[4][3], M[4][4], M[4][5], 0.0, 0.0, 0.0],
							[M[5][0], M[5][1], M[5][2], M[5][3], M[5][4], M[5][5], 0.0, 0.0, 0.0],
							[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, M[index][index], M[index][index+1], M[index][index+2]],
							[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, M[index+1][index], M[index+1][index+1], M[index+1][index+2]],
							[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, M[index+2][index], M[index+2][index+1], M[index+2][index+2]]
							])
	return M_selected

def inverseMatrix2 (M):
	invM = np.zeros((2,2))
	a = float(M[0][0])
	b = float(M[0][1])
	c = float(M[1][0])
	d = float(M[1][1])
	invM[0][0] = d/(a*d-b*c)
	invM[0][1] = -b/(a*d-b*c)
	invM[1][0] = -c/(a*d-b*c)
	invM[1][1] = a/(a*d-b*c)

	return invM