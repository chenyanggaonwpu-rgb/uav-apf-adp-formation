"""Read selected real arrays from scalar structs in our MATLAB -v7 evidence.

No SciPy dependency and no MAT writer. Format checked against SciPy's primary
reader: https://github.com/scipy/scipy/blob/main/scipy/io/matlab/_mio5_utils.pyx
This restricted reader rejects other endianness, complex arrays, and struct arrays.
"""
import math
from pathlib import Path
import struct
import zlib

def element(data,offset=0):
    first,size=struct.unpack_from('<II',data,offset)
    if first>>16:
        count=first>>16;kind=first&65535
        assert count<=4
        return kind,data[offset+4:offset+4+count],offset+8
    stop=offset+8+size;assert stop<=len(data)
    next_offset=stop if first==15 else (stop+7)//8*8
    return first,data[offset+8:stop],next_offset

def header(data):
    kind,flags,offset=element(data);assert kind==6
    flag=struct.unpack_from('<I',flags)[0]
    assert not flag&2048,'Complex arrays unsupported'
    kind,dims,offset=element(data,offset);assert kind==5
    shape=struct.unpack('<'+'i'*(len(dims)//4),dims)
    kind,name,offset=element(data,offset)
    return flag&255,shape,bytes(name).decode('utf-8'),offset

def field(data,name):
    matrix_class,shape,_,offset=header(data)
    assert matrix_class==2 and math.prod(shape)==1,'Only scalar structs supported'
    kind,length,offset=element(data,offset);assert kind==5
    width=struct.unpack_from('<i',length)[0]
    kind,names,offset=element(data,offset);assert kind==1
    fields=[bytes(names[i:i+width]).split(b'\0',1)[0].decode('utf-8') for i in range(0,len(names),width)]
    for key in fields:
        kind,child,offset=element(data,offset);assert kind==14
        if key==name:return child
    raise KeyError(name)

def variable(path,name='r'):
    data=memoryview(Path(path).read_bytes());assert bytes(data[126:128])==b'IM'
    offset=128
    while offset<len(data):
        kind,payload,offset=element(data,offset)
        if kind==15:
            decompressed=memoryview(zlib.decompress(payload))
            kind,payload,_=element(decompressed)
        assert kind==14
        _,_,found,_=header(payload)
        if found==name:return payload
    raise KeyError(name)

def numeric(data,path):
    for name in path.split('.'):data=field(data,name)
    matrix_class,shape,_,offset=header(data)
    assert matrix_class in (6,7,8,9,10,11,12,13,14,15)
    kind,values,_=element(data,offset)
    formats={1:'b',2:'B',3:'h',4:'H',5:'i',6:'I',7:'f',9:'d',12:'q',13:'Q'}
    fmt=formats[kind];count=math.prod(shape)
    assert len(values)==count*struct.calcsize(fmt)
    return shape,list(struct.unpack('<'+fmt*count,values))

def matrix_rows(shape,values):
    assert len(shape)==2
    return [[values[col*shape[0]+row] for col in range(shape[1])] for row in range(shape[0])]
