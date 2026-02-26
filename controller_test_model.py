from pydantic import BaseModel


class TestModel(BaseModel):
    test_switch: bool
    test_slider: float
    test_input: str
    test_number: float
    test_select: str
