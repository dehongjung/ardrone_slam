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

## Funcao que remove uma matriz 3x3 de uma outra matriz
## In: Matriz original, index da matriz a ser removida
## Out: Matriz final
def removeMatrix (M_old, index):
	size = len(M_old)
	index = index*3
	M = np.delete(M_old, (index, index+1, index+2), axis=0)
	M = np.delete(M, (index, index+1, index+2), axis=1)

	return M
