'''
readCSV - read a csv file
'''

def readCSV(fileName, idColumn=None, sep=','):
    '''Reads csv file in fileName.
    If idColumn is defined, use values in column as dict keys
    otherwise, put all values in list
    Define sep to use a different separator from ','
    '''
    if idColumn:
        data = {}
    else:
        data = []
    
    with open(fileName, 'r') as file:
        line = file.readline()
        headings = map(str.strip, line.split(sep))
        if idColumn:
            id = headings.index(idColumn)
        for line in file:
            tokens = map(str.strip, line.split(sep))
            if idColumn:
                row = tokens[id]
            else:
                data.append({})
                row = -1
            data[row] = {}
            for i,h in enumerate(headings):
                data[row][h] = tokens[i]
                
    return data, headings