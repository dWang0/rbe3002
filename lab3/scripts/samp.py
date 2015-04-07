def OccupyGrid(): 
    global listofOccup
    global listofUnknown
    global listofEmpty 
    global rmap #map data
    #publishers
    GCOccupied = rospy.Publisher('/GridCellsOccupied', GridCells)
    GCUnknown = rospy.Publisher('/GridCellsUnknown', GridCells)
    GCEmpty = rospy.Publisher('/GridCellsEmpty', GridCells)
    #lists to store points
    listofOccup = []
    listofUnknown = []
    listofEmpty = []
    
    for i in range(len(rmap)):
    
        row = SOMETHING + y  #current x and y spots
        column = SOMETHING + x
        
        #it's considered occupied
        if rmap[i] > 50: 
            listofOccup.append(Point(column, row, 0))
        #it's unknown
        elif MAP_DATA[i] == -1: 
            listofUnknownn.append(Point(column, row, 0))
        else: 
        #it is empty
            listofEmpty.append(Point(column, row, 0))
            
            
    GCOccupied = GridCells()
    GCOccupied.header = HEADER
    GCOccupied.cell_width = RES
    GCOccupied.cell_height = RES
    GCOccupied.cells = listofOccup
    
    GCUnknown = GridCells()
    GCUnknown.header = HEADER
    GCUnknown.cell_width = RES
    GCUnknown.cell_height = RES
    GCUnknown.cells = listofUnknown
    
    GCEmpty = GridCells()
    GCEmpty.header = HEADER
    GCEmpty.cell_width = RES
    GCEmpty.cell_height = RES
    GCEmpty.cells = listofEmpty
    GCOccupied.publish(GCOccupied)
    GCUnknown.publish(GCUnknown)
    GCEmpty.publish(GCEmpty)
    
    
    
    # question/147211/a-problem-on-rvis/#149011
    
