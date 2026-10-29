from learning_serial import parse_feedback

sample = "L=50 R=-49 X=120 Y=0"

feedback = parse_feedback(sample)

print("왼쪽 속도 :", feedback["L"])
print("오른쪽 속도 :", feedback["R"])
print("X 위치 :", feedback["X"], "mm")
print("Y 위치 :", feedback["Y"], "mm")