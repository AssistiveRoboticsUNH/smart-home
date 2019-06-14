/**
 *  NewSmartThingsRESTApiTest
 *
 *  Copyright 2019 naiqian zhang
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except
 *  in compliance with the License. You may obtain a copy of the License at:
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software distributed under the License is distributed
 *  on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License
 *  for the specific language governing permissions and limitations under the License.
 *
 */
definition(
    name: "NewSmartThingsRESTApiTest",
    namespace: "zhangnaiqian1101",
    author: "naiqian zhang",
    description: "rest api test",
    category: "",
    iconUrl: "https://s3.amazonaws.com/smartapp-icons/Convenience/Cat-Convenience.png",
    iconX2Url: "https://s3.amazonaws.com/smartapp-icons/Convenience/Cat-Convenience@2x.png",
    iconX3Url: "https://s3.amazonaws.com/smartapp-icons/Convenience/Cat-Convenience@2x.png",
    oauth: true)


preferences {
    section ("Allow external service to control contact sensor") {
    input "contacts", "capability.contactSensor", multiple: true, required: false
  }

    section ("Allow external service to control Motion sensor") {
    input "motions", "capability.motionSensor", multiple: true, required: false
  }

}

mappings {
  // motions
  path("/motions") {
    action: [
      GET: "listMotions"
    ]
  }

  // contacts
  path("/contacts") {
    action: [
      GET: "listContacts"
    ]
  }
}

//list contacts function
def listContacts() {
    def resp = []
    contacts.each {
      resp << [name: it.displayName, value: it.currentValue("contact")]
    }
    motions.each {
      resp << [name: it.displayName, value: it.currentValue("motion")]
    }
    return resp
}


//list motions function
def listMotions() {
    def resp = []
    motions.each {
      resp << [name: it.displayName, id:it.id, value: it.currentValue("motion"), temp: it.currentValue("temperature")]
    }
    return resp
}


def installed() {
	log.debug "Installed with settings: ${settings}"

	initialize()
}

def updated() {
	log.debug "Updated with settings: ${settings}"

	unsubscribe()
	initialize()
}

def initialize() {
	// TODO: subscribe to attributes, devices, locations, etc.
}

// TODO: implement event handlers
