from sqlalchemy import create_engine


class Base:

    """
    Class to comunicate with postgreSQL, at the initial are creating if not exist already two table which are response
    to keep data from Arduino and final counted result of force.
    """

    tests_coutner = 0

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        DATABASE_URI = 'postgres://postgres:postgres@localhost:5432/data_droptest'
        self.db = create_engine(DATABASE_URI)

        self.db.execute("CREATE TABLE IF NOT EXISTS table_results (id integer PRIMARY KEY, date text, "
                        "reflection_travel numeric, reflection_time numeric, fall_time numeric, "
                        "force_method1 numeric, force_method2 numeric)")

        self.db.execute("CREATE TABLE IF NOT EXIST table_data_measured (id integer PRIMARY KEY, time numeric, "
                        "upper_sensor numeric, lower_sensor numeric, acceleroemter_x numeric, accelerometer_y numeric, "
                        "acceleroemter_z numeric")

        self.get_test_counter()

    def get_test_counter(self):
        if self.db.execute("SELECT MAX(id) FROM table_results") is not None:
            self.tests_coutner = self.db.execute("SELECT MAX(id) FROM table_results") + 1
        else:
            self.tests_coutner = 0

    def table_result_adding(self,  date, reflection_travel, reflection_time, fall_time, force_method1,
                            force_method2):
        self.db.execute("INSERT INTO table_results (id, date, reflection_travel, reflection_time, fall_time, "
                        "force_method1, force_method2) VALUES ('{0}', '{1}', '{2}', '{3}', '{4}', '{5}', '{6}')"
                        .format(self.tests_coutner, date, reflection_travel, reflection_time,
                                fall_time, force_method1, force_method2))

    def table_result_read(self, table_id):
        return self.db.execute("SELECT * FROM table_results WHERE id={}".format(table_id))

    def table_data_adding(self, measured_time, upper_sensor, lower_sensor, accelerometer_x, accelerometer_y,
                          accelerometer_z):
        self.db.execute("INSERT INTO table_data_measured (id, time, upper_sensor, lower_sensor, accelerometer_x, "
                        "accelerometer_y, accelerometer_z) VALUES ('{0}', '{1}', '{2}', '{3}', '{4}', '{5}', '{6}')"
                        .format(self.tests_coutner, measured_time, upper_sensor, lower_sensor,
                                accelerometer_x, accelerometer_y, accelerometer_z))

    def table_data_read(self, table_id):
        return self.db.execute("SELECT * FROM table_data_measured WHERE id={}".format(table_id))