package pathFinder;

import map.Coordinate;
import map.PathMap;

import java.util.*;

public class DijkstraPathFinder implements PathFinder
{
   private LinkedList<Coordinate> sourceCoords;
   private LinkedList<Coordinate> wayPoints;
   private LinkedList<Coordinate> destCoords;

   LinkedList<LinkedList<Coordinate>> allShortestPaths = new LinkedList<LinkedList<Coordinate>>();
   LinkedList<LinkedList<Coordinate>> allShortestWaypoints = new LinkedList<LinkedList<Coordinate>>();
   LinkedList<Coordinate> shortestPath = new LinkedList<Coordinate>();
   LinkedList<Coordinate> shortestWaypointPath = new LinkedList<Coordinate>();

   private boolean isVisited[][];
   private PathMap map;
   Coordinate preCoord;
   Coordinate newCoord;
   private int numCoordExplored = 0;
   int cost = 0;

   public DijkstraPathFinder(PathMap map) {
	   this.map = map;

      sourceCoords = new LinkedList<Coordinate>(map.originCells);
      wayPoints = new LinkedList<Coordinate>(map.waypointCells);
      destCoords = new LinkedList<Coordinate>(map.destCells);
//    this a reference for when comparing multipul start coordinates
      Coordinate currStartCoord;

//    if waypoints exist
      if(wayPoints.size()!=0) {
//       find the 2 nearest source/waypoint for all combinations
    	   allShortestPaths = allPathsBetween2Lists(sourceCoords, wayPoints);
//       select the 2 nearest points
    	   shortestWaypointPath = smallestPath(allShortestPaths);
//       add it as the first leg of the path
         allShortestWaypoints.addLast(new LinkedList<Coordinate>(shortestWaypointPath));

         cost = 0;
         preCoord = shortestWaypointPath.getLast();
         currStartCoord = new Coordinate(preCoord.getRow(),preCoord.getColumn());
         cost = map.cells[preCoord.getRow()][preCoord.getColumn()].getTerrainCost();
         currStartCoord.setTerrainCost(cost);

//       compute the next nearest waypoint
    	   while(wayPoints.size()!=0) {
        	   allShortestPaths = allPathsBetweenCoordinateAndList(currStartCoord, wayPoints);
        	   shortestWaypointPath = smallestPath(allShortestPaths);
//          add current path to overall path from source
        	   allShortestWaypoints.addLast(new LinkedList<Coordinate>(shortestWaypointPath));
            cost = 0;
            preCoord = shortestWaypointPath.getLast();
        	   currStartCoord = new Coordinate(preCoord.getRow(),preCoord.getColumn());
            cost = map.cells[preCoord.getRow()][preCoord.getColumn()].getTerrainCost();
            currStartCoord.setTerrainCost(cost);
            wayPoints.remove(shortestWaypointPath.getLast());
    	  }

         cost = 0;
         preCoord = allShortestWaypoints.getLast().getLast();
         currStartCoord = new Coordinate(preCoord.getRow(),preCoord.getColumn());
         cost = map.cells[preCoord.getRow()][preCoord.getColumn()].getTerrainCost();
         currStartCoord.setTerrainCost(cost);
//       compute the shortest path between last waypoint and single or collection of destination/s
    	   allShortestPaths = allPathsBetweenCoordinateAndList(currStartCoord, destCoords);
    	   shortestWaypointPath = smallestPath(allShortestPaths);
    	   allShortestWaypoints.addLast(new LinkedList<Coordinate>(shortestWaypointPath));

      }
      else {
//       no waypoints, compute the shortest distance between all start and end coords
         allShortestPaths = allPathsBetween2Lists(sourceCoords, destCoords);
         shortestPath = smallestPath(allShortestPaths);
      }



   } // end of DijkstraPathFinder()

   @Override
   public List<Coordinate> findPath() {
      LinkedList<Coordinate> path = new LinkedList<Coordinate>();
//    if there were waypoints, piece together the path in orde
      if(allShortestWaypoints.size()!=0) {
         for(int i=0;i<allShortestWaypoints.size()-1;i++){
            allShortestWaypoints.get(i).removeLast();
         }
         for(int i=0;i<allShortestWaypoints.size();i++){
            path.addAll(new LinkedList<Coordinate>(allShortestWaypoints.get(i)));
         }
      }
//    if no way points then simply return the path
      else {
    	   path = shortestPath;
      }
      return path;
   } // end of findPath()

   private List<Coordinate> getNeighbours(Coordinate u){
//    create a list of all visitable neighbours that havent been explored
	   List<Coordinate> neigh = new ArrayList<Coordinate>();

	   int r;int c;
      r = u.getRow();
      c = u.getColumn() - 1;
      if((map.isIn(r, c))&&!isVisited[r][c]&&map.isPassable(r, c)){
         neigh.add(new Coordinate(r, c));
      }

      r = u.getRow();
      c = u.getColumn() + 1;
      if((map.isIn(r, c))&&!isVisited[r][c]&&map.isPassable(r, c)){
    	  neigh.add(new Coordinate(r, c));
      }

      r = u.getRow()+1;
      c = u.getColumn();
      if((map.isIn(r, c))&&!isVisited[r][c]&&map.isPassable(r, c)){
         neigh.add(new Coordinate(r, c));
      }

      r = u.getRow()-1;
      c = u.getColumn();
      if((map.isIn(r, c))&&!isVisited[r][c]&&map.isPassable(r, c)){
    	  neigh.add(new Coordinate(r, c));
      }
      return neigh;
   }
// return an adjList of all paths between 2 lists
   private LinkedList<LinkedList<Coordinate>> allPathsBetween2Lists(LinkedList<Coordinate>origin, LinkedList<Coordinate>destination) {
	   List<Coordinate> neighbours = new LinkedList<Coordinate>();
	   LinkedList<LinkedList<Coordinate>> pathways = new LinkedList<LinkedList<Coordinate>>();
	   LinkedList<LinkedList<Coordinate>> smallestPathways = new LinkedList<LinkedList<Coordinate>>();

	   Coordinate currSource;
	   Coordinate currDest;

	   for(Coordinate orig : origin){
    		for(Coordinate dest : destination){
		    	isVisited = new boolean[map.sizeR][map.sizeC];
//		    	initialisation w first coord and its neighbours
			   currSource = orig;
            currDest = dest;
			   isVisited[currSource.getRow()][currSource.getColumn()] = true;
//			   add source to adjlist
			   LinkedList<Coordinate>init = new LinkedList<Coordinate>();
			   init.add(currSource);
			   pathways.add(init);

//				this will find the shortest path between 2 points
			   while(!currSource.equals(currDest)) {
//				   find neighbours and update weights
					neighbours = getNeighbours(currSource);
//					create an arraylist based on source for each neighbour appended
					int found = -1;
					for(int j=0;j<pathways.size();j++) {
						   if(pathways.get(j).getLast() == currSource){
							   found = j;
						   }
					}

					   LinkedList<Coordinate> addList = new LinkedList<Coordinate>(pathways.get(found));
					   for(int j=0;j<neighbours.size();j++) {
                     cost = 0;
                     preCoord = neighbours.get(j);
                     newCoord = new Coordinate(preCoord.getRow(),preCoord.getColumn());
                     cost = map.cells[preCoord.getRow()][preCoord.getColumn()].getTerrainCost();
                     newCoord.setTerrainCost(cost);

						   addList.add(newCoord);
						   pathways.add(new LinkedList<Coordinate>(addList));

						   addList.removeLast();
					   }
                  numCoordExplored++;
					   neighbours.clear();

					   for(int j=0;j<pathways.size();j++) {
					  	   if(pathways.get(j).getLast().equals(currSource)&&!pathways.get(j).getLast().equals(currDest)) {
							   pathways.remove(j);
						   }
					   }

					   int smallestPath = 0;

					   for(int j=0;j<pathways.size();j++) {
						   int listSize = 0;
						   int otherSize = 0;

						   for (int k=0;k<pathways.get(j).size();k++) {
							   listSize+=pathways.get(j).get(k).getTerrainCost();
						   }
						   for (int k=0;k<pathways.get(smallestPath).size();k++) {
							   otherSize+=pathways.get(smallestPath).get(k).getTerrainCost();
						   }

						   if(listSize<otherSize) {
							   smallestPath = j;
						   }
					   }

					   currSource = pathways.get(smallestPath).getLast();
					   isVisited[currSource.getRow()][currSource.getColumn()] = true;
			      }// end while loop finding shortest path

				   List<Coordinate> path = new ArrayList<Coordinate>();

			      int smallestPath = 0;
			      for(int j=0;j<pathways.size();j++) {
					   int listSize = 0;
					   int otherSize = 0;

					   for (int k=0;k<pathways.get(j).size();k++) {
						   listSize+=pathways.get(j).get(k).getTerrainCost();
					   }
					   for (int k=0;k<pathways.get(smallestPath).size();k++) {
						   otherSize+=pathways.get(smallestPath).get(k).getTerrainCost();
					   }

					   if(listSize<otherSize) {
						   smallestPath = j;
					   }
				   }

			      path = pathways.get(smallestPath);
			      smallestPathways.add(new LinkedList<Coordinate>(path));
			      pathways.clear();

		      }//end source iteration
	      }//end dest iteration

	   return smallestPathways;
   }

   private LinkedList<LinkedList<Coordinate>> allPathsBetweenCoordinateAndList(Coordinate originCoord, LinkedList<Coordinate>destination) {
	   LinkedList<Coordinate> origin = new LinkedList<Coordinate>();
	   origin.add(originCoord);
      return this.allPathsBetween2Lists(origin,destination);
   }

   private LinkedList<Coordinate> smallestPath(LinkedList<LinkedList<Coordinate>> adjList){
	   LinkedList<Coordinate> path = new LinkedList<Coordinate>();
	   int smallestPath = 0;
      for(int j=0;j<adjList.size();j++) {
		  	int listSize = 0;
			int otherSize = 0;

		   for (int k=0;k<adjList.get(j).size();k++) {
			   listSize+=adjList.get(j).get(k).getTerrainCost();
		   }
		   for (int k=0;k<adjList.get(smallestPath).size();k++) {
			   otherSize+=adjList.get(smallestPath).get(k).getTerrainCost();
		   }

		   if(listSize<otherSize) {
			   smallestPath = j;
		   }
	   }
      path = adjList.get(smallestPath);
	   return path;
   }
   @Override
   public int coordinatesExplored() {
      return numCoordExplored;
   } // end of cellsExplored()
} // end of class DijsktraPathFinder
