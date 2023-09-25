package com.swrobotics.shufflelog.util;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImDouble;
import imgui.type.ImString;

import java.util.*;
import java.util.function.BiFunction;
import java.util.function.Function;

public final class ExpressionInput {
    private static final class State {
        ImString inputBuffer = null;
        boolean wasActive = false;
        boolean exprValid = true;
    }

    private final Map<Integer, State> inputStorage;
    private final Set<Integer> inactive;
    private ImString inactivePreview;

    public ExpressionInput() {
        inputStorage = new HashMap<>();
        inactive = new HashSet<>();
        inactivePreview = new ImString(128);
    }

    private enum TokenType {
        Number, Name,
        Add, Subtract, Multiply, Divide, Exponent,
        OpenParen, CloseParen,

        End, // Represents hitting end of input
        Error // Represents invalid character found
    }

    private static final class Token {
        TokenType type;
        String str;

        public Token(TokenType type) {
            this.type = type;
        }

        public Token(TokenType type, String str) {
            this.type = type;
            this.str = str;
        }
    }

    private static final class Lexer {
        String input;
        int startIdx, currentIdx;

        Lexer(String input) {
            this.input = input;
            startIdx = 0;
            currentIdx = 0;
        }

        Token nextNumber() {
            boolean hasDigitsBeforeDecimal = false;
            while (currentIdx < input.length() && Character.isDigit(input.charAt(currentIdx))) {
                currentIdx++;
                hasDigitsBeforeDecimal = true;
            }
            if (currentIdx < input.length() && input.charAt(currentIdx) == '.') {
                currentIdx++;

                // Make sure it's not just a single .
                if (!hasDigitsBeforeDecimal && (currentIdx >= input.length() || !Character.isDigit(input.charAt(currentIdx))))
                    return new Token(TokenType.Error);

                while (currentIdx < input.length() && Character.isDigit(input.charAt(currentIdx)))
                    currentIdx++;
            }
            return new Token(TokenType.Number, input.substring(startIdx, currentIdx));
        }

        Token nextName() {
            while (currentIdx < input.length() && Character.isLetterOrDigit(input.charAt(currentIdx)))
                currentIdx++;
            return new Token(TokenType.Name, input.substring(startIdx, currentIdx));
        }

        void repeat() {
            currentIdx = startIdx;
        }

        Token nextToken() {
            startIdx = currentIdx;
            while (startIdx < input.length() && Character.isWhitespace(input.charAt(startIdx)))
                startIdx++;
            if (startIdx == input.length())
                return new Token(TokenType.End);
            currentIdx = startIdx;

            char c = input.charAt(currentIdx);
            switch (c) {
                case '+': currentIdx++; return new Token(TokenType.Add);
                case '-': currentIdx++; return new Token(TokenType.Subtract);
                case '*': currentIdx++; return new Token(TokenType.Multiply);
                case '/': currentIdx++; return new Token(TokenType.Divide);
                case '^': currentIdx++; return new Token(TokenType.Exponent);

                case '(': currentIdx++; return new Token(TokenType.OpenParen);
                case ')': currentIdx++; return new Token(TokenType.CloseParen);

                default:
                    if (Character.isDigit(c) || c == '.') {
                        return nextNumber();
                    } else if (Character.isLetter(c)) {
                        return nextName();
                    } else {
                        // There was an invalid token, fail to parse
                        return new Token(TokenType.Error);
                    }
            }
        }
    }

    private enum Operator {
        Add(1, false, (a, b) -> a + b),
        Subtract(1, false, (a, b) -> a - b),
        Multiply(2, false, (a, b) -> a * b),
        Divide(2, false, (a, b) -> a / b),
        Exponent(3, true, Math::pow),
        Negate(4, true, (a, b) -> -b);

        final int precedence;
        final boolean rightAssoc;
        final BiFunction<Double, Double, Double> eval;

        Operator(int precedence, boolean rightAssoc, BiFunction<Double, Double, Double> eval) {
            this.precedence = precedence;
            this.rightAssoc = rightAssoc;
            this.eval = eval;
        }
    }

    private static final Map<TokenType, Operator> operators = new EnumMap<>(TokenType.class);
    private static final Map<String, Function<Double, Double>> functions = new HashMap<>();
    static {
        operators.put(TokenType.Add, Operator.Add);
        operators.put(TokenType.Subtract, Operator.Subtract);
        operators.put(TokenType.Multiply, Operator.Multiply);
        operators.put(TokenType.Divide, Operator.Divide);
        operators.put(TokenType.Exponent, Operator.Exponent);

        functions.put("sin", Math::sin);
        functions.put("cos", Math::cos);
        functions.put("tan", Math::tan);
        functions.put("asin", Math::asin);
        functions.put("acos", Math::acos);
        functions.put("atan", Math::atan);
    }

    private void err(String message) {
        ImGui.textColored(1.0f, 0.0f, 0.0f, 1.0f, message);
    }

    private void applyOp(Stack<Double> valStack, Operator op) {
        Double right = valStack.pop();
        Double left = valStack.pop();
        valStack.push(op.eval.apply(left, right));
    }

    private Double parseExpr(Lexer lexer, boolean insideParen) {
        Stack<Operator> operStack = new Stack<>();
        Stack<Double> valStack = new Stack<>();

        Token token;
        boolean prevWasOp = true;
        TokenType prevType = null;
        loop: while ((token = lexer.nextToken()).type != TokenType.End) {
            switch (token.type) {
                case Number:
                    if (prevType == TokenType.Number) {
                        err("Cannot have multiple numbers in a row");
                        return null;
                    }
                    if (!prevWasOp)
                        operStack.push(Operator.Multiply);
                    prevWasOp = false;
                    valStack.push(Double.parseDouble(token.str));
                    break;
                case Name:
                    if (!prevWasOp)
                        operStack.push(Operator.Multiply);
                    prevWasOp = false;
                    if (token.str.equals("pi")) {
                        valStack.push(Math.PI);
                        break;
                    }
                    if (lexer.nextToken().type != TokenType.OpenParen) {
                        err("Missing ( for function " + token.str);
                        return null;
                    }
                    Function<Double, Double> fn = functions.get(token.str);
                    if (fn == null) {
                        err("Unknown function " + token.str);
                        return null;
                    }
                    Double input = parseExpr(lexer, true);
                    if (input == null)
                        return null; // Don't report error, the sub-expression already did
                    valStack.push(fn.apply(input));
                    TokenType nextType = lexer.nextToken().type;
                    if (nextType != TokenType.CloseParen) {
                        if (nextType == TokenType.End)
                            break loop; // Act as if closed at end of expr
                        return null;
                    }
                    break;

                case OpenParen:
                    if (!prevWasOp)
                        operStack.push(Operator.Multiply);
                    prevWasOp = false;
                    input = parseExpr(lexer, true);
                    if (input == null)
                        return null;
                    valStack.push(input);
                    nextType = lexer.nextToken().type;
                    if (nextType != TokenType.CloseParen) {
                        if (nextType == TokenType.End)
                            break loop; // Act as if closed at end of expr
                        return null;
                    }
                    break;

                case Error: {
                    err("Unexpected token");
                    return null;
                }

                case CloseParen:
                    if (insideParen) {
                        lexer.repeat();
                        break loop;
                    }
                    // Act as if there was open paren at start of expression
                    break;

                default:
                    Operator op = operators.get(token.type);
                    if (op == null) {
                        lexer.repeat();
                        break loop;
                    }
                    if (op == Operator.Subtract && prevWasOp) {
                        op = Operator.Negate;
                        valStack.push(0.0); // Dummy left-hand side for negate
                    }

                    prevWasOp = true;
                    Operator op2;
                    while (!operStack.empty()) {
                        op2 = operStack.peek();
                        if ((!op.rightAssoc && op.precedence == op2.precedence) || op.precedence < op2.precedence) {
                            operStack.pop();
                            if (valStack.size() < 2) {
                                err("Missing left or right operand");
                                return null;
                            }
                            applyOp(valStack, op2);
                        } else {
                            break;
                        }
                    }
                    operStack.push(op);
                    break;
            }
            prevType = token.type;
        }

        // Reached the end of the expression
        while (!operStack.empty()) {
            if (valStack.size() < 2) {
                err("Missing left or right operand");
                return null;
            }
            applyOp(valStack, operStack.pop());
        }
        if (valStack.isEmpty()) {
            err("No value");
            return null;
        }
        return valStack.pop();
    }

    public Double tryParse(String expr) {
        return parseExpr(new Lexer(expr), false);
    }

    public void inputDouble(String label, ImDouble value) {
        // Get stored state
        Integer id = ImGui.getID(label);
        State state = inputStorage.computeIfAbsent(id, (l) -> new State());
        inactive.remove(id); // Mark state as retained for next frame

        ImString str;
        if (state.wasActive) {
            str = state.inputBuffer;
        } else {
            str = inactivePreview;
            str.set(String.format("%.6f", value.get())); // Preview stored value
        }

        boolean colorInvalid = state.wasActive && !state.exprValid;
        if (colorInvalid)
            ImGui.pushStyleColor(ImGuiCol.FrameBg, 0.3f, 0.1f, 0.1f, 1.0f);
        ImGui.inputText(label, str);
        if (colorInvalid)
            ImGui.popStyleColor();

        boolean active = ImGui.isItemActive();
        if (active) {
            if (!state.wasActive) {
                // Move the preview buffer into state, and use a new buffer for remaining previews
                state.inputBuffer = inactivePreview;
                inactivePreview = new ImString(128);
            }

            // Attempt to parse current value
            Double newVal = tryParse(state.inputBuffer.get());
            if (state.exprValid = newVal != null) {
                value.set(newVal);
            }
        }
        state.wasActive = active;
    }

    public void newFrame() {
        for (Integer s : inactive) {
            inputStorage.remove(s);
        }
        inactive.clear();
        inactive.addAll(inputStorage.keySet());
    }
}
