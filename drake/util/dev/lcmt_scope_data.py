"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

import cStringIO as StringIO
import struct

class lcmt_scope_data(object):
    __slots__ = ["scope_id", "num_points", "linespec", "resetOnXval", "xdata", "ydata"]

    def __init__(self):
        self.scope_id = 0
        self.num_points = 0
        self.linespec = ""
        self.resetOnXval = False
        self.xdata = 0
        self.ydata = 0

    def encode(self):
        buf = StringIO.StringIO()
        buf.write(lcmt_scope_data._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qi", self.scope_id, self.num_points))
        __linespec_encoded = self.linespec.encode('utf-8')
        buf.write(struct.pack('>I', len(__linespec_encoded)+1))
        buf.write(__linespec_encoded)
        buf.write("\0")
        buf.write(struct.pack(">bdd", self.resetOnXval, self.xdata, self.ydata))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = StringIO.StringIO(data)
        if buf.read(8) != lcmt_scope_data._get_packed_fingerprint():
            raise ValueError("Decode error")
        return lcmt_scope_data._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = lcmt_scope_data()
        self.scope_id, self.num_points = struct.unpack(">qi", buf.read(12))
        __linespec_len = struct.unpack('>I', buf.read(4))[0]
        self.linespec = buf.read(__linespec_len)[:-1].decode('utf-8')
        self.resetOnXval, self.xdata, self.ydata = struct.unpack(">bdd", buf.read(17))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if lcmt_scope_data in parents: return 0
        newparents = parents + [lcmt_scope_data]
        tmphash = (0xce3bf247025afc2f) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff 
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None
    
    def _get_packed_fingerprint():
        if lcmt_scope_data._packed_fingerprint is None:
            lcmt_scope_data._packed_fingerprint = struct.pack(">Q", lcmt_scope_data._get_hash_recursive([]))
        return lcmt_scope_data._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

