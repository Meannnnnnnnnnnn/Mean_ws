#!/usr/bin/env python3

from flask import Flask, render_template, request, jsonify, g
import subprocess
import signal
import os
import time
import sqlite3

app = Flask(__name__)

DATABASE_DIR = os.path.join(os.path.dirname(__file__), "static")
DATABASE = os.path.join(DATABASE_DIR, "database.db")

if not os.path.exists(DATABASE_DIR):
    os.makedirs(DATABASE_DIR)

def get_db():
    db = getattr(g, '_database', None)
    if db is None:
        db = g._database = sqlite3.connect(DATABASE)
    return db

@app.teardown_appcontext
def close_connection(exception):
    db = getattr(g, '_database', None)
    if db is not None:
        db.close()

def init_db():
    with app.app_context():
        try:
            c = get_db().cursor()
            c.execute("CREATE TABLE IF NOT EXISTS maps (id INTEGER PRIMARY KEY, name TEXT NOT NULL)")
            c.close()
        except sqlite3.Error as e:
            print(e)

init_db()  # Ensure this is called at the start of the application

class ROSLaunchProcess:
    process_navigation = None
    process_mapping = None

    @classmethod
    def start_navigation(cls, docmap):
        cls.stop_navigation()  # Ensure previous instance is stopped
        try:
            cls.process_navigation = subprocess.Popen([
                "roslaunch", "--wait", "navigation_server", 
                "turtlebot3_navigation.launch", "map_file:="+os.getcwd()+"/static/"+docmap+".yaml"
            ])
        except Exception as e:
            print(f"Failed to start navigation: {e}")

    @classmethod
    def stop_navigation(cls):
        if cls.process_navigation and cls.process_navigation.poll() is None:
            cls.process_navigation.send_signal(signal.SIGINT)
            cls.process_navigation.wait()

    @classmethod
    def start_mapping(cls):
        cls.stop_mapping()  # Ensure previous instance is stopped
        try:
            cls.process_mapping = subprocess.Popen([
                "roslaunch", "--wait", "navigation_server", "turtlebot3_slam.launch"
            ])
        except Exception as e:
            print(f"Failed to start mapping: {e}")

    @classmethod
    def stop_mapping(cls):
        if cls.process_mapping and cls.process_mapping.poll() is None:
            cls.process_mapping.send_signal(signal.SIGINT)
            cls.process_mapping.wait()

@app.route('/')
def index():
    init_db()
    try:
        with get_db():
            c = get_db().cursor()
            c.execute("SELECT * FROM maps")
            data = c.fetchall()
            c.close()
    except sqlite3.Error as e:
        print(e)
        data = []
    return render_template('index.html', title='Index', map=data)

@app.route('/index/<variable>', methods=['GET', 'POST'])
def themainroute(variable):
    if variable == "navigation-precheck":
        try:
            with get_db():
                c = get_db().cursor()
                c.execute("SELECT count(*) FROM maps")
                k = c.fetchall()[0][0]
                c.close()
                return jsonify(mapcount=k)
        except sqlite3.Error as e:
            print(e)
    elif variable == "gotonavigation":
        mapname = request.get_data().decode('utf-8')
        ROSLaunchProcess.start_navigation(mapname)
        return "success"

@app.route('/navigation', methods=['GET', 'POST'])
def navigation():
    try:
        with get_db():
            c = get_db().cursor()
            c.execute("SELECT * FROM maps")
            data = c.fetchall()
            c.close()
    except sqlite3.Error as e:
        print(e)
        data = []
    return render_template('navigation.html', map=data)

@app.route('/navigation/deletemap', methods=['POST'])
def deletemap():
    mapname = request.get_data().decode('utf-8')
    os.system("rm -rf " + os.getcwd() + "/static/" + mapname + ".yaml " + os.getcwd() + "/static/" + mapname + ".png " + os.getcwd() + "/static/" + mapname + ".pgm")
    try:
        with get_db():
            c = get_db().cursor()
            c.execute("DELETE FROM maps WHERE name=?", (mapname,))
            c.close()
    except sqlite3.Error as e:
        print(e)
    return "successfully deleted map"

@app.route("/navigation/<variable>", methods=['GET', 'POST'])
def gotomapping(variable):
    if variable == "index":
        ROSLaunchProcess.start_mapping()
    elif variable == "gotomapping":
        ROSLaunchProcess.stop_navigation()
        time.sleep(2)
        ROSLaunchProcess.start_mapping()
    return "success"

@app.route("/navigation/loadmap", methods=['POST'])
def navigation_properties():
    mapname = request.get_data().decode('utf-8')
    ROSLaunchProcess.stop_navigation()
    time.sleep(5)
    ROSLaunchProcess.start_navigation(mapname)
    return "success"

@app.route("/navigation/stop", methods=['POST'])
def stop():
    os.system("rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}")
    return "stopped the robot"

@app.route('/mapping')
def mapping():
    try:
        with get_db():
            c = get_db().cursor()
            c.execute("SELECT * FROM maps")
            data = c.fetchall()
            c.close()
    except sqlite3.Error as e:
        print(e)
        data = []
    return render_template('mapping.html', title='Mapping', map=data)

@app.route("/mapping/cutmapping", methods=['POST'])
def killnode():
    ROSLaunchProcess.stop_mapping()
    return "killed the mapping node"

@app.route("/mapping/savemap", methods=['POST'])
def savemap():
    mapname = request.get_data().decode('utf-8')
    os.system("rosrun map_server map_saver -f " + os.path.join(os.getcwd(), "static", mapname))
    os.system("convert " + os.getcwd() + "/static/" + mapname + ".pgm " + os.getcwd() + "/static/" + mapname + ".png")
    try:
        with get_db():
            c = get_db().cursor()
            c.execute("INSERT INTO maps (name) VALUES (?)", (mapname,))
            c.close()
    except sqlite3.Error as e:
        print(e)
    return "success"

@app.route("/shutdown", methods=['POST'])
def shutdown():
    os.system("shutdown now")
    return "shutting down the robot"

@app.route("/restart", methods=['POST'])
def restart():
    os.system("restart now")
    return "restarting the robot"

if __name__ == '__main__':
    app.run(debug=False)
