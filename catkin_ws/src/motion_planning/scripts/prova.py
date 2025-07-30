import spacy
from collections import defaultdict

def get_ingredient_list_from_user():
    print("\nHello! What kind of burger would you like?")
    print("\n\tAvailable ingredients are: bread, meat, cheese, tomato, and salad")
    nlp = spacy.load("en_core_web_sm") 
    user_input = input("\nBurger description: ")

    ingredients = {"bread", "meat", "cheese", "tomato", "salad"}
    doc = nlp(user_input.lower())
    
    explicitly_included = set()
    explicitly_excluded = set()

    for token in doc:
        word = token.text.lower()
        if word in ingredients:
            negated = False

            # Controllo figli (es. "no cheese")
            for child in token.children:
                if child.dep_ in {"neg", "det"} and child.text.lower() in {"no", "not", "any", "without"}:
                    negated = True

            # Controllo parole a sinistra (es. "without meat")
            for left in token.lefts:
                if left.text.lower() in {"no", "not", "any", "without"}:
                    negated = True

            # Controllo anche antenati (es. "I don't want cheese")
            for ancestor in token.ancestors:
                for child in ancestor.children:
                    if child.dep_ == "neg":
                        negated = True

            if negated:
                explicitly_excluded.add(word)
            else:
                explicitly_included.add(word)

    if explicitly_included:
        return list(explicitly_included)
    else:
        return list(ingredients - explicitly_excluded)

listina=get_ingredient_list_from_user()
print(listina)
