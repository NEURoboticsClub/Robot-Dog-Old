import openai
import config
import string
from collections import Counter

openai.api_key = config.DevelopmentConfig.OPENAI_KEY



def generateChatResponse(prompt):

    messages = [{"role": "system", "content": "You are a robot dog. i will give you commands in english and you will convert them to twist commands in code that can be understood by a robot. Give me linear and angular code for x,y,x axes in indented format. Always give the output in code format and do not inclide any other text. If it is not a valid command, juse respond as usual."},
                {"role":"user", "content": prompt}]

    question = {}
    question['role'] = 'user'
    question['content'] = prompt
    messages.append(question)

    response = openai.ChatCompletion.create(model="gpt-3.5-turbo", messages=messages)
    print(response.choices[0].message.content)

    try:
        answer = response['choices'][0]['message']['content'].replace('\n', '<br>')
    except:
        answer = 'Oops lets try again'
    
    return answer