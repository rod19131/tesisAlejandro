""" =========================================================================
% FUNCIONES VARIAS
% =========================================================================
% Autor: Andrea Maybell Peña Echeverría
% Última modificación: 01/09/2019
% ========================================================================= """

# Librerías importadas
from controller import Robot, Motor, Supervisor, Node
import numpy as np
import math

  
def Fmatrix(f,r):
#FMATRIX Retorna matriz de formación deseada
#   f = número de formación / número de grafo
#   r = nivel de rigidez (de 1 a 8)

	d0 = 2*math.sqrt(0.75)
	b0 = math.sqrt((1.5*d0)**2 + 0.25)
	b1 = math.sqrt(d0**2 + 4)
	b2 = math.sqrt((0.5*d0)**2 + 2.5**2)

	## MATRICES
	# matrices de adyacencia grafo mínimamente rígido
	d1 = np.array([[0, 1, 1, 0, 0, 0, 0, 0, 0, 0],
			  [1, 0, 1, 0, 1, 1, 0, 0, 0, 0],
			  [1, 1, 0, 1, 1, 0, 0, 0, 0, 0],
			  [0, 0, 1, 0, 1, 0, 1, 1, 0, 0],
			  [0, 1, 1, 1, 0, 1, 0, 1, 1, 0],
			  [0, 1, 0, 0, 1, 0, 0, 0, 1, 1],
			  [0, 0, 0, 1, 0, 0, 0, 1, 0, 0],
			  [0, 0, 0, 1, 1, 0, 1, 0, 0, 0],
			  [0, 0, 0, 0, 1, 1, 0, 0, 0, 1],
			  [0, 0, 0, 0, 0, 1, 0, 0, 1, 0]])

	   
	d2 = np.array([[0, 1, 1, 0, 1, 0, 0, 0, 0, 0],
			  [1, 0, 0, 0, 1, 1, 0, 0, 0, 0],
			  [1, 0, 0, 1, 1, 0, 0, 0, 0, 0],
			  [0, 0, 1, 0, 1, 0, 1, 1, 0, 0],
			  [1, 1, 1, 1, 0, 1, 0, 1, 0, 0],
			  [0, 1, 0, 0, 1, 0, 0, 1, 0, 1],
			  [0, 0, 0, 1, 0, 0, 0, 1, 1, 0],
			  [0, 0, 0, 1, 1, 1, 1, 0, 1, 1],
			  [0, 0, 0, 0, 0, 0, 1, 1, 0, 1],
			  [0, 0, 1, 0, 0, 1, 0, 1, 1, 0]])

	d3 = np.array([[0, 1, 1, 0, 0, 0, 0, 0, 0, 0],
			  [1, 0, 1, 1, 0, 0, 0, 0, 0, 0],
			  [1, 1, 0, 1, 1, 0, 0, 0, 0, 0],
			  [0, 1, 1, 0, 1, 1, 0, 0, 0, 0],
			  [0, 0, 1, 1, 0, 1, 1, 0, 0, 0],
			  [0, 0, 0, 1, 1, 0, 1, 1, 0, 0],
			  [0, 0, 0, 0, 1, 1, 0, 1, 1, 0],
			  [0, 0, 0, 0, 0, 1, 1, 0, 1, 1],
			  [0, 0, 0, 0, 0, 0, 1, 1, 0, 1],
			  [0, 0, 0, 0, 0, 0, 0, 1, 1, 0]])
	  
	# matrices de adyacencia todos los nodos conectados
	d2m1 = np.array([[0, 1, 1, 1, 0, 0, 0, 0, 0, 0],
			[1, 0, 1, 1, 1, 1, 1, 0, 0, 0],
			[1, 1, 0, 0, 1, 0, 0, 0, 0, 0],
			[1, 1, 0, 0, 0, 1, 0, 0, 0, 0],
			[0, 1, 1, 0, 0, 0, 1, 0, 1, 0],
			[0, 1, 0, 1, 0, 0, 1, 1, 0, 0],
			[0, 1, 0, 0, 1, 1, 0, 1, 1, 1],
			[0, 0, 0, 0, 0, 1, 1, 0, 0, 1],
			[0, 0, 0, 0, 1, 0, 1, 0, 0, 1],
			[0, 0, 0, 0, 0, 0, 1, 1, 1, 0]])
		
	# matrices considerando "segundo grado de adyacencia"
	d2m2 = np.array([[0, 1, 1, 0, 1, 0, 0, 2, 0, 0],
			   [1, 0, 0, 2, 1, 1, 0, 0, 0, 2],
			   [1, 0, 0, 1, 1, 2, 2, 0, 0, 0],
			   [0, 2, 1, 0, 1, 0, 1, 1, 0, 2],
			   [1, 1, 1, 1, 0, 1, 0, 1, 2, 0],
			   [0, 1, 2, 0, 1, 0, 2, 1, 0, 1],
			   [0, 0, 2, 1, 0, 2, 0, 1, 1, 0],
			   [2, 0, 0, 1, 1, 1, 1, 0, 1, 1],
			   [0, 0, 0, 0, 2, 0, 1, 1, 0, 1],
			   [0, 2, 0, 2, 0, 1, 0, 1, 1, 0]])

	d3m1 = np.array([[0, 1, 1, 0, 2, 0, 0, 0, 0, 0],
				[1, 0, 1, 1, 0, 2, 0, 0, 0, 0],
				[1, 1, 0, 1, 1, 0, 2, 0, 0, 0],
				[0, 1, 1, 0, 1, 1, 0, 2, 0, 0],
				[2, 0, 1, 1, 0, 1, 1, 0, 2, 0],
				[0, 2, 0, 1, 1, 0, 1, 1, 0, 2],
				[0, 0, 2, 0, 1, 1, 0, 1, 1, 0],
				[0, 0, 0, 2, 0, 1, 1, 0, 1, 1],
				[0, 0, 0, 0, 2, 0, 1, 1, 0, 1],
				[0, 0, 0, 0, 0, 2, 0, 1, 1, 0]])
	  
	dm1 = np.array([[0, 1, 1, 2, 0, 2, 0, 0, 0, 0],
			  [1, 0, 1, 0, 1, 1, 0, 2, 0, 2],
			  [1, 1, 0, 1, 1, 0, 2, 0, 2, 0],
			  [2, 0, 1, 0, 1, 2, 1, 1, 0, 0],
			  [0, 1, 1, 1, 0, 1, 0, 1, 1, 0],
			  [2, 1, 0, 2, 1, 0, 0, 0, 1, 1],
			  [0, 0, 2, 1, 0, 0, 0, 1, 0, 0],
			  [0, 2, 0, 1, 1, 0, 1, 0, 0, 0],
			  [0, 0, 2, 0, 1, 1, 0, 0, 0, 1],
			  [0, 2, 0, 0, 0, 1, 0, 0, 1, 0]])

	dm2 = np.array([[0, 1, 1, 2, 0, 2, 0, 0, 0, 0],
			  [1, 0, 1, 0, 1, 1, 0, 2, 0, 2],
			  [1, 1, 0, 1, 1, 0, 2, 0, 2, 0],
			  [2, 0, 1, 0, 1, 2, 1, 1, 0, 0],
			  [0, 1, 1, 1, 0, 1, 0, 1, 1, 0],
			  [2, 1, 0, 2, 1, 0, 0, 0, 1, 1],
			  [0, 0, 2, 1, 0, 0, 0, 1, 0, 0],
			  [0, 2, 0, 1, 1, 0, 1, 0, 1, 0],
			  [0, 0, 2, 0, 1, 1, 0, 1, 0, 1],
			  [0, 2, 0, 0, 0, 1, 0, 0, 1, 0]])
	  
	# matrices considerando "tercer grado de adyacencia"  
	dm3 = np.array([[0, 1, 1, 2, 0, 2, 3, 0, 0, 3],
			  [1, 0, 1, 0, 1, 1, 0, 2, 0, 2],
			  [1, 1, 0, 1, 1, 0, 2, 0, 2, 0],
			  [2, 0, 1, 0, 1, 2, 1, 1, 0, 0],
			  [0, 1, 1, 1, 0, 1, 0, 1, 1, 0],
			  [2, 1, 0, 2, 1, 0, 0, 0, 1, 1],
			  [3, 0, 2, 1, 0, 0, 0, 1, 2, 3],
			  [0, 2, 0, 1, 1, 0, 1, 0, 1, 2],
			  [0, 0, 2, 0, 1, 1, 2, 1, 0, 1],
			  [3, 2, 0, 0, 0, 1, 3, 2, 1, 0]])
		  
	d2m3 = np.array([[0, 1, 1, 0, 1, 0, 0, 2, 3, 0],
			   [1, 0, 0, 2, 1, 1, 0, 0, 0, 2],
			   [1, 0, 0, 1, 1, 2, 2, 0, 0, 0],
			   [0, 2, 1, 0, 1, 0, 1, 1, 0, 2],
			   [1, 1, 1, 1, 0, 1, 0, 1, 2, 0],
			   [0, 1, 2, 0, 1, 0, 2, 1, 0, 1],
			   [0, 0, 2, 1, 0, 2, 0, 1, 1, 0],
			   [2, 0, 0, 1, 1, 1, 1, 0, 1, 1],
			   [3, 0, 0, 0, 2, 0, 1, 1, 0, 1],
			   [0, 2, 0, 2, 0, 1, 0, 1, 1, 0]])
	 
	dr1 = np.array([[0, 1, 1, 2, d0, 2, 3, 0, 0, 3],
		   [1, 0, 1, d0, 1, 1, 0, 2, d0, 2],
		   [1, 1, 0, 1, 1, d0, 2, d0, 2, 0],
		   [2, d0, 1, 0, 1, 2, 1, 1, d0, 0],
		   [d0, 1, 1, 1, 0, 1, d0, 1, 1, d0],
		   [2, 1, d0, 2, 1, 0, 0, d0, 1, 1],
		   [3, 0, 2, 1, d0, 0, 0, 1, 2, 3],
		   [0, 2, d0, 1, 1, d0, 1, 0, 1, 2],
		   [0, d0, 2, d0, 1, 1, 2, 1, 0, 1],
		   [3, 2, 0, 0, d0, 1, 3, 2, 1, 0]])
	   
	dr2 = np.array([[0, 1, 1, 2, d0, 2, 3, b0, b0, 3],
		   [1, 0, 1, d0, 1, 1, 0, 2, d0, 2],
		   [1, 1, 0, 1, 1, d0, 2, d0, 2, 0],
		   [2, d0, 1, 0, 1, 2, 1, 1, d0, 0],
		   [d0, 1, 1, 1, 0, 1, d0, 1, 1, d0],
		   [2, 1, d0, 2, 1, 0, 0, d0, 1, 1],
		   [3, 0, 2, 1, d0, 0, 0, 1, 2, 3],
		   [b0, 2, d0, 1, 1, d0, 1, 0, 1, 2],
		   [b0, d0, 2, d0, 1, 1, 2, 1, 0, 1],
		   [3, 2, 0, 0, d0, 1, 3, 2, 1, 0]])
	   
	dr3 = np.array([[0, 1, 1, 2, d0, 2, 3, b0, b0, 3],
		   [1, 0, 1, d0, 1, 1, b1, 2, d0, 2],
		   [1, 1, 0, 1, 1, d0, 2, d0, 2, b1],
		   [2, d0, 1, 0, 1, 2, 1, 1, d0, 0],
		   [d0, 1, 1, 1, 0, 1, d0, 1, 1, d0],
		   [2, 1, d0, 2, 1, 0, 0, d0, 1, 1],
		   [3, b1, 2, 1, d0, 0, 0, 1, 2, 3],
		   [b0, 2, d0, 1, 1, d0, 1, 0, 1, 2],
		   [b0, d0, 2, d0, 1, 1, 2, 1, 0, 1],
		   [3, 2, b1, 0, d0, 1, 3, 2, 1, 0]])

	# Matriz completamente rígida
	dr4 = np.array([[0, 1, 1, 2, d0, 2, 3, b0, b0, 3],
			   [1, 0, 1, d0, 1, 1, b1, 2, d0, 2],
			   [1, 1, 0, 1, 1, d0, 2, d0, 2, b1],
			   [2, d0, 1, 0, 1, 2, 1, 1, d0, b2],
			   [d0, 1, 1, 1, 0, 1, d0, 1, 1, d0],
			   [2, 1, d0, 2, 1, 0, b2, d0, 1, 1],
			   [3, b1, 2, 1, d0, b2, 0, 1, 2, 3],
			   [b0, 2, d0, 1, 1, d0, 1, 0, 1, 2],
			   [b0, d0, 2, d0, 1, 1, 2, 1, 0, 1],
			   [3, 2, b1, b2, d0, 1, 3, 2, 1, 0]])
		   
	d2r = np.array([[0, 1, 1, d0, 1, d0, b2, 2, 3, b2],
			   [1, 0, d0, 2, 1, 1, b1, d0, b2, 2],
			   [1, d0, 0, 1, 1, 2, 2, d0, b2, b1],
			   [d0, 2, 1, 0, 1, d0, 1, 1, d0, 2],
			   [1, 1, 1, 1, 0, 1, d0, 1, 2, d0],
			   [d0, 1, 2, d0, 1, 0, 2, 1, d0, 1],
			   [b2, b1, 2, 1, d0, 2, 0, 1, 1, d0],
			   [2, d0, d0, 1, 1, 1, 1, 0, 1, 1],
			   [3, b2, b2, d0, 2, d0, 1, 1, 0, 1],
			   [b2, 2, b1, 2, d0, 1, d0, 1, 1, 0]])
	
	
	MM = [d1, dm1, dm2, dm3, dr1, dr2, dr3, dr4, 
		  d2, d2m1, d2m2, d2m3, d2m3, d2m3, d2m3, d2r, 
		  d3, d3m1, d3m1, d3m1, d3m1, d3m1, d3m1, d3m1]
		  
	mat = (8*(f-1))+(r-1)
	
	return MM[mat]


