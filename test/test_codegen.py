import pytest
import script.codegen as c


def test_direction_matrix():
    m = c.gen_dir_mat()
    assert m[6, 9] == c.W
    assert m[6, 10] == c.E
    assert m[0, 1] == c.N
    assert m[1, 0] == c.N
    assert m[15, 9] == c.S
