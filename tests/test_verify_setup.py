from verify_setup import check_modules


def test_check_modules():
    assert check_modules() == []
