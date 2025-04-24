from src.graphs.DataSet import DataSet


class Graph:
    def __init__(self, name: str, dots_file_name: str, data_set_dimension: int):
        self.name = name
        self.dots_file_name = dots_file_name
        self.data_set = DataSet(data_set_dimension)

    def get_name(self) -> str:
        return self.name

    def get_data_set(self) -> DataSet:
        return self.data_set

    def write_dots_to_file(self):
        try:
            with open(self.dots_file_name, 'w') as f:
                f.write(self.data_set.dot_to_string())
        except Exception as e:
            raise e
        except IndexError as e:
            print("No dots to write")
