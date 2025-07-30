import spacy
from collections import Counter

import spacy
from collections import Counter
import spacy
from collections import Counter

def get_ingredient_list_from_user():
    print("\nHello! What kind of burger would you like?")
    print("\n\tAvailable ingredients are: bread, meat, cheese, tomato, and salad")
    
    nlp = spacy.load("en_core_web_sm") 
    user_input = input("\nBurger description: ").lower()

    ingredients = {"bread", "meat", "cheese", "tomato", "salad"}
    doc = nlp(user_input)

    ingredient_counts = Counter()
    excluded_ingredients = set()
    explicitly_mentioned = set()

    # Flag per identificare frasi di tipo "only meat", "just cheese", ecc.
    exclusive_mode = any(word in user_input for word in ["only", "just", "nothing but", "solo", "esclusivamente"])

    for token in doc:
        word = token.text.lower()
        if word in ingredients:
            negated = False
            double = False

            # NEGAZIONI
            for child in token.children:
                if child.dep_ in {"neg", "det"} and child.text in {"no", "not", "any", "without", "except"}:
                    negated = True
            for left in token.lefts:
                if left.text in {"no", "not", "any", "without", "except"}:
                    negated = True
            for ancestor in token.ancestors:
                for child in ancestor.children:
                    if child.dep_ == "neg":
                        negated = True

            # QUANTITÀ DOPPIA
            for modifier in token.lefts:
                if modifier.text in {"double", "extra", "more", "a lot of", "tons of"}:
                    double = True

            if negated:
                excluded_ingredients.add(word)
            else:
                explicitly_mentioned.add(word)
                ingredient_counts[word] += 2 if double else 1

    # Comportamento corretto per aggiungere gli altri ingredienti:
    if exclusive_mode or len(explicitly_mentioned) > 1:
        # caso esclusivo o più ingredienti menzionati: uso solo quelli
        pass
    elif len(explicitly_mentioned) == 1:
        # caso un solo ingrediente menzionato esplicitamente (es. "double cheese")
        # aggiungo tutti gli altri che non sono esclusi e non sono menzionati esplicitamente
        for ing in ingredients - excluded_ingredients - explicitly_mentioned:
            ingredient_counts[ing] = 1
    else:
        # nessun ingrediente menzionato, includo tutto tranne esclusi
        for ing in ingredients - excluded_ingredients:
            ingredient_counts[ing] = 1

    # Costruisco lista finale con ripetizioni
    final_ingredients = []
    for ing, count in ingredient_counts.items():
        final_ingredients.extend([ing] * count)

    return final_ingredients
listina=get_ingredient_list_from_user()
print(listina)
