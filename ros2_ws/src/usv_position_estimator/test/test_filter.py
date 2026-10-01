import numpy as np
from usv_position_estimator.filter import DelayedPositionFilter, quaternion_matrix


def test_delay_replay_matches_in_order():
    ordered, delayed = DelayedPositionFilter(gate=1e9), DelayedPositionFilter(gate=1e9)
    for f in [ordered, delayed]:
        f.add(1., 'imu', [0.2,0.])
        f.add(1., 'position', [0.,0.])
    for i in range(1,101):
        t=1+i*.01
        ordered.add(t,'imu',[0.2,0.])
        delayed.add(t,'imu',[0.2,0.])
        if i==50:
            ordered.add(t,'position',[0.03,0.])
    delayed.add(1.5,'position',[0.03,0.])
    np.testing.assert_allclose(delayed.current[1],ordered.current[1],atol=1e-10)
    np.testing.assert_allclose(delayed.current[2],ordered.current[2],atol=1e-10)


def test_stationary_and_covariance():
    f=DelayedPositionFilter()
    f.add(1.,'imu',[0.,0.]);f.add(1.,'position',[2.,3.])
    for i in range(1,101): f.add(1+i*.01,'imu',[0.,0.])
    np.testing.assert_allclose(f.current[1][:2],[2.,3.],atol=1e-6)
    assert np.linalg.eigvalsh(f.current[2]).min()>=-1e-10


def test_old_and_outlier_rejected():
    f=DelayedPositionFilter(history=.2)
    f.add(1.,'imu',[0.,0.]);f.add(1.,'position',[0.,0.])
    f.add(2.,'imu',[0.,0.]);f.add(2.1,'imu',[0.,0.])
    assert not f.add(.9,'position',[0.,0.])
    assert not f.add(2.1,'position',[1000.,1000.])


def test_gravity_rotation():
    r=quaternion_matrix([np.sin(.2),0.,0.,np.cos(.2)])
    body=r.T@np.array([0.,0.,9.80665])
    np.testing.assert_allclose(r@body-[0.,0.,9.80665],[0.,0.,0.],atol=1e-10)


def test_large_absolute_origin_does_not_create_velocity():
    f=DelayedPositionFilter()
    f.add(1.,'imu',[0.,0.])
    f.add(1.1,'imu',[0.,0.])
    assert f.add(1.05,'position',[500000.,4000000.])
    np.testing.assert_allclose(f.current[1][:4],[500000.,4000000.,0.,0.])
