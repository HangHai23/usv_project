import json
from usv_udp_position.protocol import ReceiverState


def packet(seq=1, frame=1):
    return dict(protocol='air2s.vision',version=1,session='a'*32,seq=seq,
        sent_unix_ms=1790000000100,ttl_ms=500,age_ms=30.,fresh=True,
        observation=dict(frame=frame,coord='b'*32,field_valid=True,field_status='locked',
            size_m=[20.,11.],goals=[],ball=dict(state='lost',xy_m=None,score=0.),
            boats=[dict(id=0,xy_m=[1.,2.],tag_heading_deg=0.,possession='unknown')],
            processed_unix_ms=1790000000080))


def test_expiry_duplicate_and_repeated_frame():
    s=ReceiverState()
    assert s.accept(json.dumps(packet()).encode(),10.)
    assert not s.accept(json.dumps(packet()).encode(),10.1)
    assert s.accept(json.dumps(packet(2)).encode(),10.2)
    assert s.current(10.48) is None


def test_reset_and_invalid():
    s=ReceiverState();p=packet()
    assert s.accept(json.dumps(p).encode(),1.)
    p['seq']=2;p['observation']['coord']='c'*32
    assert not s.accept(json.dumps(p).encode(),1.1)
    assert s.reset_required and s.current(1.1) is None
    for data in [b'{}', b'[]', b'bad', b'x'*1401]:
        assert not ReceiverState().accept(data,1.)


def test_nonfinite_and_invalid_packet():
    p=packet();p['observation']['boats'][0]['xy_m'][0]=float('nan')
    assert not ReceiverState().accept(json.dumps(p).encode(),1.)
    s=ReceiverState();p=packet();assert s.accept(json.dumps(p).encode(),1.)
    p.update(seq=2,fresh=False,observation=None)
    assert s.accept(json.dumps(p).encode(),1.1)
    assert s.current(1.1) is None
