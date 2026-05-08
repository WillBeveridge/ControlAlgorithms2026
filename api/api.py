from flask import Flask, request, jsonify, url_for, redirect
import json
import atexit
from waitress import serve
import numpy as np
import logging

NUM_ROBOTS = 6

app = Flask(__name__)
app.config["DEBUG"] = False


# open import and close db.json
readFile = open("db.json", 'r')
data = json.load(readFile)
readFile.close()

# adding exit routine to save the db.json file with the current states (not really necessary but is nice for debugging)


def exit_routine():
    print("Terminating server")
    writeFile = open("db.json", "w")
    writeFile.write(json.dumps(data, indent=4, sort_keys=True))
    writeFile.close()


atexit.register(exit_routine)

# default route


@app.route("/", methods=["GET"])
def home():
    return "Hello, World!"

# agent position route


@app.route("/agents/<id>", methods=["GET", "PUT"])
def agentsReq(id):
    id = int(id)
    if request.method == "GET":
        if id > 0 and id <= len(data["agents"]):
            return jsonify(data["agents"][id-1])
    if request.method == "PUT":
        if id > 0 and id <= len(data["agents"]):
            data["agents"][id-1] = request.json
            return jsonify(data["agents"][id-1])


# robot position estimate route


@app.route("/agentsLocal/<id>", methods=["GET", "PUT"])
def agentsLocalReq(id):
    id = int(id)
    if request.method == "GET":
        if id > 0 and id <= len(data["agentsLocal"]):
            return jsonify(data["agentsLocal"][id-1])
    if request.method == "PUT":
        if id > 0 and id <= len(data["agentsLocal"]):
            data["agentsLocal"][id-1] = request.json
            return jsonify(data["agentsLocal"][id-1])

# agent ready route with id


@app.route("/agentReady/<id>", methods=["GET", "PUT"])
def agentReadyReq(id):
    id = int(id)
    if request.method == "GET":
        if id > 0 and id <= len(data["agentReady"]):
            return jsonify(data["agentReady"][id-1])
    if request.method == "PUT":
        if id > 0 and id <= len(data["agentReady"]):
            data["agentReady"][id-1] = request.json
            return jsonify(data["agentReady"][id-1])

# agent ready route without id


@app.route("/agentReady", methods=["GET"])
def agentReadyNoIDReq():
    return jsonify(data["agentReady"])

# agentGo route with id 1


@app.route("/agentGo/<id>", methods=["GET", "PUT"])
def agentGoReq(id):
    id = int(id)
    if request.method == "GET":
        if 0 < id and id < (NUM_ROBOTS + 1):
            return jsonify(data["agentGo"][id-1])
    if request.method == "PUT":
        if 0 < id and id < (NUM_ROBOTS + 1):
            data["agentGo"][id-1] = request.json
            return jsonify(data["agentGo"][id-1])

# goal1 route with id


@app.route("/goal1/<id>", methods=["GET", "PUT", "DELETE", "POST", "HEAD"])
def goal1Req(id):
    id = int(id)
    if request.method == "GET":
        if id > 0 and id <= len(data["goal1"]):
            return jsonify(data["goal1"][id-1])
    if request.method == "PUT":
        if id > 0 and id <= len(data["goal1"]):
            data["goal1"][id-1] = request.json
            return jsonify(data["goal1"][id-1])
        else:
            data["goal1"].append(request.json)
            return jsonify(data["goal1"][-1])
    if request.method == "DELETE":
        if id > 0 and id <= len(data["goal1"]):
            data["goal1"].pop(id-1)
            return jsonify(data["goal1"])
        else:
            # 404
            return "", 404
    if request.method == "POST":
        if id > 0:
            data["goal1"].append(request.json)
            return jsonify(data["goal1"][id-1])
    if request.method == "HEAD":
        if id > 0 and id <= len(data["goal1"]):
            return jsonify(data["goal1"][id-1])
        else:
            # return a 404 error
            return "", 404

# route for goal2 with id


@app.route("/goal2/<id>", methods=["GET", "PUT", "DELETE", "POST", "HEAD"])
def goal2Req(id):
    id = int(id)
    if request.method == "GET":
        if id > 0 and id <= len(data["goal2"]):
            return jsonify(data["goal2"][id-1])
    if request.method == "PUT":
        if id > 0 and id <= len(data["goal2"]):
            data["goal2"][id-1] = request.json
            return jsonify(data["goal2"][id-1])
        else:
            data["goal2"].append(request.json)
            return jsonify(data["goal2"][-1])
    if request.method == "DELETE":
        if id > 0 and id <= len(data["goal2"]):
            data["goal2"].pop(id-1)
            return jsonify(data["goal2"])
        else:
            # return a 404 error
            return "", 404
    if request.method == "POST":
        if id > 0:
            data["goal2"].append(request.json)
            return jsonify(data["goal2"][id-1])
    if request.method == "HEAD":
        if id > 0 and id <= len(data["goal2"]):
            return jsonify(data["goal2"][id-1])
        else:
            # return a 404 error
            return "", 404

# route for goal3 with id


@app.route("/goal3/<id>", methods=["GET", "PUT", "DELETE", "POST", "HEAD"])
def goal3Req(id):
    id = int(id)
    if request.method == "GET":
        if id > 0 and id <= len(data["goal3"]):
            return jsonify(data["goal3"][id-1])
    if request.method == "PUT":
        if id > 0 and id <= len(data["goal3"]):
            data["goal3"][id-1] = request.json
            return jsonify(data["goal3"][id-1])
        else:
            data["goal3"].append(request.json)
            return jsonify(data["goal3"][-1])
    if request.method == "DELETE":
        if id > 0 and id <= len(data["goal3"]):
            data["goal3"].pop(id-1)
            return jsonify(data["goal3"])
        else:
            return "", 404
    if request.method == "POST":
        if id > 0:
            data["goal3"].append(request.json)
            return jsonify(data["goal3"][id-1])
    if request.method == "HEAD":
        if id > 0 and id <= len(data["goal3"]):
            return jsonify(data["goal3"][id-1])
        else:
            # return a 404 error
            return "", 404


# route for goal4 with id


@app.route("/goal4/<id>", methods=["GET", "PUT", "DELETE", "POST", "HEAD"])
def goal4Req(id):
    id = int(id)
    if request.method == "GET":
        if id > 0 and id <= len(data["goal4"]):
            return jsonify(data["goal4"][id-1])
    if request.method == "PUT":
        if id > 0 and id <= len(data["goal4"]):
            data["goal4"][id-1] = request.json
            return jsonify(data["goal4"][id-1])
        else:
            data["goal4"].append(request.json)
            return jsonify(data["goal4"][-1])
    if request.method == "DELETE":
        if id > 0 and id <= len(data["goal4"]):
            data["goal4"].pop(id-1)
            return jsonify(data["goal4"])
        else:
            return "", 404
    if request.method == "POST":
        if id > 0:
            data["goal4"].append(request.json)
            return jsonify(data["goal4"][id-1])
    if request.method == "HEAD":
        if id > 0 and id <= len(data["goal4"]):
            return jsonify(data["goal4"][id-1])
        else:
            # return a 404 error
            return "", 404

# route for goal5 with id


@app.route("/goal5/<id>", methods=["GET", "PUT", "DELETE", "POST", "HEAD"])
def goal5Req(id):
    id = int(id)
    if request.method == "GET":
        if id > 0 and id <= len(data["goal5"]):
            return jsonify(data["goal5"][id-1])
    if request.method == "PUT":
        if id > 0 and id <= len(data["goal5"]):
            data["goal5"][id-1] = request.json
            return jsonify(data["goal5"][id-1])
        else:
            data["goal5"].append(request.json)
            return jsonify(data["goal5"][-1])
    if request.method == "DELETE":
        if id > 0 and id <= len(data["goal5"]):
            data["goal5"].pop(id-1)
            return jsonify(data["goal5"])
        else:
            return "", 404
    if request.method == "POST":
        if id > 0:
            data["goal5"].append(request.json)
            return jsonify(data["goal5"][id-1])
    if request.method == "HEAD":
        if id > 0 and id <= len(data["goal5"]):
            return jsonify(data["goal5"][id-1])
        else:
            # return a 404 error
            return "", 404


# route for goal6 with id


@app.route("/goal6/<id>", methods=["GET", "PUT", "DELETE", "POST", "HEAD"])
def goal6Req(id):
    id = int(id)
    if request.method == "GET":
        if id > 0 and id <= len(data["goal6"]):
            return jsonify(data["goal6"][id-1])
    if request.method == "PUT":
        if id > 0 and id <= len(data["goal6"]):
            data["goal6"][id-1] = request.json
            return jsonify(data["goal6"][id-1])
        else:
            data["goal6"].append(request.json)
            return jsonify(data["goal6"][-1])
    if request.method == "DELETE":
        if id > 0 and id <= len(data["goal6"]):
            data["goal6"].pop(id-1)
            return jsonify(data["goal6"])
        else:
            return "", 404
    if request.method == "POST":
        if id > 0:
            data["goal6"].append(request.json)
            return jsonify(data["goal6"][id-1])
    if request.method == "HEAD":
        if id > 0 and id <= len(data["goal6"]):
            return jsonify(data["goal6"][id-1])
        else:
            # return a 404 error
            return "", 404




# route for accepting 3 by 3 position array and splitting it to the 3 agents


@app.route("/allPos/1", methods=["PUT"])
def allPos():
    if request.method == "PUT":
        data["allPos"] = request.json
        pos = np.array(request.json["pos"])
        for i in range(NUM_ROBOTS):
            data["agents"][i]["position"] = pos[i, :].tolist()
        return jsonify(data["allPos"])


# use this for debugging (if you need to print the requests) (not as fast as using waitress)
app.run(host="192.168.0.101", port=3000, debug=False, threaded=False)#, processes=3)
#serve(app, host="192.168.0.100", port=3000, threads=4)
