
def state0(activity): #Default state that returns to if it does not encounter a good letter
    if activity=='a':
        return statea
    else:
        return state0

def statea(activity):#The state that it returns to whenever there is an A
    if activity=='b':
        return stateb
    elif activity=='a':
        return statea
    else:
        return state0

def stateb(activity):#The state that it goes to if there was an a followed by a b
    if(activity=='c'):
        return statec
    elif(activity=='a'):
        return statea
    else:
        return state0

def statec(activity):#The state that it goes too if you had an abc
    if (activity=='d'):
        print("Congrats abcd is contained within the string!")
        return stated
    elif (activity=='a'):
        return statea
    else:
        return state0

def stated(activity):#The state that it goes too if you had an abcd
    if(activity=='a'):
        return statea
    else:
        return state0


state_dictionary = {
    state0: "Waiting for A",
    statea: "Received A waiting for B",
    stateb: "Received B waiting for C",
    statec: "Received C waiting for D",
    stated: "Received D waiting for A"
}





state = state0    # initial state as pointer to state0 function
user_input=input("Please input a string ")
input_length=len(user_input)
dummy_char='z'
for activity in user_input: #loops for each character in the userinput
    # Get current activity
    new_state = state(activity)  # launch state machine
    state = new_state   # update the next state
    #print(f"The state is {state}")

print("Done with state machine")










