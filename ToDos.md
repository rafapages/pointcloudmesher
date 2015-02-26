X - Hacer una búsqueda de vecinos cercanos entre la malla y la point cloud para descartar vértices de la malla lejos
  de la propia nube
	  X - Ahora el valor que busca de distancia es random, hay que automatizar el valor
      X - Los parámetros de entrada y salida son todavía dudosos... hay que ver cómo ponerlos
	  X - IMPORTANTE: hay que hacer cleanup de vértices que no se usan en la PolygonMesh: http://www.pcl-users.org/Delete-faces-and-vertices-from-PolyMesh-td4020090.html	
	
  - Segmentador de cilindros

X - Limpiar la nube de mierdaca
		

  - Automatizar valores de parámetros

  - Mejorar el sistema de segmentación de planos: probar a descartar puntos si su normal (previamente calculada) no está alineada con la del plano en el que ha sido incluido

NOTA: Para manejar los vértices y triángulos de una PolygonMesh: http://www.pcl-users.org/Access-pcl-PolygonMesh-triangles-data-td4025718.html