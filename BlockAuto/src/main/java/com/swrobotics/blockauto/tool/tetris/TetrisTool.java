package com.swrobotics.blockauto.tool.tetris;

import com.swrobotics.blockauto.BlockAuto;
import com.swrobotics.blockauto.tool.ViewportTool;
import imgui.ImGui;
import imgui.ImVec2;
import org.lwjgl.glfw.GLFW;
import processing.core.PGraphics;

public final class TetrisTool extends ViewportTool {
    public static final TetrominoShape[] SHAPES = {
            new TetrominoShape(TileState.LIGHTBLUE, new Mino(-1, 0), new Mino(0, 0), new Mino(1, 0), new Mino(2, 0)), // I
            new TetrominoShape(TileState.BLUE, new Mino(-1, -1), new Mino(-1, 0), new Mino(0, 0), new Mino(1, 0)), // J
            new TetrominoShape(TileState.ORANGE, new Mino(-1, 0), new Mino(0, 0), new Mino(1, 0), new Mino(1, -1)), // L
            new TetrominoShape(TileState.YELLOW, new Mino(0, -1), new Mino(1, -1), new Mino(0, 0), new Mino(1, 0)), // O
            new TetrominoShape(TileState.GREEN, new Mino(0, -1), new Mino(1, -1), new Mino(-1, 0), new Mino(0, 0)), // S
            new TetrominoShape(TileState.PURPLE, new Mino(0, -1), new Mino(-1, 0), new Mino(0, 0), new Mino(1, 0)), // T
            new TetrominoShape(TileState.RED, new Mino(-1, -1), new Mino(0, -1), new Mino(0, 0), new Mino(1, 0)) // Z
    };

    private static final int maxRotationAttempts = 5;
    private static final Kick[][] kickDef = {
            {new Kick(0, 0), new Kick( 0, 0), new Kick( 0, 0), new Kick(0,  0), new Kick( 0,  0)},
            {new Kick(0, 0), new Kick( 1, 0), new Kick( 1, 1), new Kick(0, -2), new Kick( 1, -2)},
            {new Kick(0, 0), new Kick( 0, 0), new Kick( 0, 0), new Kick(0,  0), new Kick( 0,  0)},
            {new Kick(0, 0), new Kick(-1, 0), new Kick(-1, 1), new Kick(0, -2), new Kick(-1, -2)}
    };
    private static final Kick[][] kickI = {
            {new Kick( 0,  0), new Kick(-1,  0), new Kick( 2,  0), new Kick(-1,  0), new Kick( 2,  0)},
            {new Kick(-1,  0), new Kick( 0,  0), new Kick( 0,  0), new Kick( 0, -1), new Kick( 0,  2)},
            {new Kick(-1, -1), new Kick( 1, -1), new Kick(-2, -1), new Kick( 1,  0), new Kick(-2,  0)},
            {new Kick( 0, -1), new Kick( 0, -1), new Kick( 0, -1), new Kick( 0,  1), new Kick( 0, -2)}
    };
    private static final Kick[][] kickO = { // Only one per row because it can never fail
            {new Kick( 0, 0)},
            {new Kick( 0, 1)},
            {new Kick(-1, 1)},
            {new Kick(-1, 0)}
    };
    // Same order as shapes are defined
    private static final Kick[][][] kickTables = {kickI, kickDef, kickDef, kickO, kickDef, kickDef, kickDef};

    public static final int pixelsPerTile = 20;

    private static final int matrixOffsetX = 6;


    private static final int NORMAL_DROP_TIME = 60;
    private static final int FAST_DROP_TIME = 3;

    private KeyRepeat leftRepeat = new KeyRepeat();
    private KeyRepeat rightRepeat = new KeyRepeat();

    private GameState state = GameState.PLAYING;

    private PieceRandomizer randomizer = new PieceRandomizer();
    private Matrix matrix = new Matrix();
    private TetrominoContext ctx, preview, hold = null;

    private boolean hasHeld = false;
    private int dropTimer;
    private DropSpeed dropSpeed;
    private boolean aboutToLand = false;

    private int linesCleared = 0;

    private static final class EdgeDetector {
        private boolean prevVal;
        private boolean val;

        public void set(boolean val) {
            prevVal = this.val;
            this.val = val;
        }

        public boolean isRising() {
            return val && !prevVal;
        }

        public boolean isFalling() {
            return !val && prevVal;
        }
    }

    private final EdgeDetector upEdge = new EdgeDetector();
    private final EdgeDetector downEdge = new EdgeDetector();
    private final EdgeDetector leftEdge = new EdgeDetector();
    private final EdgeDetector rightEdge = new EdgeDetector();
    private final EdgeDetector spaceEdge = new EdgeDetector();
    private final EdgeDetector shiftEdge = new EdgeDetector();

    public TetrisTool(BlockAuto log) {
        super(log, "Tetris");
        beginGame();
    }

    public static void drawMino(PGraphics g, int x, int y, int col) {
        g.stroke(0);
        g.fill(col);
        g.rect(x * pixelsPerTile, y * pixelsPerTile, pixelsPerTile, pixelsPerTile);
    }

    private int getLevel() {
        return linesCleared / 15;
    }

    private int getDropTime() {
        return dropSpeed == DropSpeed.NORMAL ? Math.max(FAST_DROP_TIME, (int) (NORMAL_DROP_TIME * Math.pow(getLevel() + 1, -0.6f)) - 5) : FAST_DROP_TIME;
    }

    private void beginGame() {
        linesCleared = 0;

        matrix.clear();
        dropSpeed = DropSpeed.NORMAL;

        preview = new TetrominoContext();
        preview.shapeIdx = randomizer.getNext();
        preview.x = Matrix.matrixWidth + 2;
        preview.y = 2;

        hold = null;

        spawnNextPiece();
    }

    private void step() {
        int left = leftRepeat.getPressed();
        int right = rightRepeat.getPressed();
        boolean moved = false;
        for (int i = 0; i < left; i++)
            moved |= tryMove(-1, 0);
        for (int i = 0; i < right; i++)
            moved |= tryMove(1, 0);

        // For now assume we're always at 60 fps

        ctx.y += 1;
        aboutToLand = matrix.collides(ctx);
        ctx.y -= 1;
        if (aboutToLand && moved) {
            dropTimer = getDropTime();
        }

        if (dropTimer <= 0) {
            dropTimer = getDropTime();
            if (aboutToLand) {
                // Hit bottom
                linesCleared += matrix.plot(ctx);
                spawnNextPiece();
            } else {
                ctx.y += 1;
            }
        }
        dropTimer--;
    }

    private void spawnPiece(int shape) {
        ctx = new TetrominoContext();
        ctx.shapeIdx = shape;
        ctx.x = 5;
        ctx.y = 1;

        dropTimer = getDropTime();
        hasHeld = false;

        if (matrix.collides(ctx)) {
            state = GameState.GAME_OVER;
        }
    }

    private void spawnNextPiece() {
        spawnPiece(preview.shapeIdx);
        preview.shapeIdx = randomizer.getNext();
    }

    private void hold() {
        if (hasHeld)
            return;

        Integer prevHold = hold == null ? null : hold.shapeIdx;

        hold = new TetrominoContext();
        hold.shapeIdx = ctx.shapeIdx;
        hold.x = -4;
        hold.y = 2;

        if (prevHold == null) {
            spawnNextPiece();
        } else {
            spawnPiece(prevHold);
        }

        hasHeld = true;
    }

    private void tryRotate() {
        Kick[][] kickTable = kickTables[ctx.shapeIdx];

        Rotation from = ctx.rot;
        Kick[] fromKicks = kickTable[ctx.rot.kickIdx];
        ctx.rot = ctx.rot.next();
        Kick[] toKicks = kickTable[ctx.rot.kickIdx];

        for (int i = 0; i < maxRotationAttempts; i++) {
            Kick fromKick = fromKicks[i];
            Kick toKick = toKicks[i];

            int dx = fromKick.dx - toKick.dx;
            int dy = fromKick.dy - toKick.dy;
            if (tryMove(dx, dy))
                return;
        }

        // All attempts failed, go back
        ctx.rot = from;
    }

    private boolean tryMove(int dx, int dy) {
        ctx.x += dx;
        ctx.y += dy;
        boolean collide = matrix.collides(ctx);
        if (collide) {
            ctx.x -= dx;
            ctx.y -= dy;
        }
        return !collide;
    }

    private TetrominoContext getHardDropPos() {
        // Copy current context
        TetrominoContext drop = new TetrominoContext();
        drop.shapeIdx = ctx.shapeIdx;
        drop.x = ctx.x;
        drop.y = ctx.y;
        drop.rot = ctx.rot;

        // Move it down until it collides
        do {
            drop.y++;
        } while (!matrix.collides(drop));
        drop.y--;

        return drop;
    }

    @Override
    protected void drawGuiContent() {
        float titleBar = ImGui.getFrameHeight();
        ImVec2 pad = ImGui.getStyle().getWindowPadding();
        ImGui.setWindowSize(pad.x * 2 + (Matrix.matrixWidth + 12) * pixelsPerTile, titleBar + pad.y * 2 + Matrix.matrixHeight * pixelsPerTile + 1);
        super.drawGuiContent();
    }

    @Override
    protected void drawViewportContent(PGraphics g) {
        boolean input = ImGui.isWindowFocused();
        upEdge.set(input && ImGui.isKeyDown(GLFW.GLFW_KEY_UP));
        leftEdge.set(input && ImGui.isKeyDown(GLFW.GLFW_KEY_LEFT));
        rightEdge.set(input && ImGui.isKeyDown(GLFW.GLFW_KEY_RIGHT));
        downEdge.set(input && ImGui.isKeyDown(GLFW.GLFW_KEY_DOWN));
        spaceEdge.set(input && ImGui.isKeyDown(GLFW.GLFW_KEY_SPACE));
        shiftEdge.set(input && ImGui.isKeyDown(GLFW.GLFW_KEY_LEFT_SHIFT));

        if (state == GameState.GAME_OVER) {
            if (spaceEdge.isRising()) {
                state = GameState.PLAYING;
                beginGame();
            }
        } else if (state == GameState.PLAYING) {
            if (spaceEdge.isRising()) {
                ctx = getHardDropPos();
                dropTimer = 0;
            }
            if (upEdge.isRising()) {
                tryRotate();
                if (aboutToLand)
                    dropTimer = getDropTime();
            }
            if (leftEdge.isRising())
                leftRepeat.setDown(true);
            if (rightEdge.isRising())
                rightRepeat.setDown(true);
            if (downEdge.isRising()) {
                dropSpeed = DropSpeed.FAST;
                dropTimer = 0;
            }
            if (shiftEdge.isRising())
                hold();

            if (downEdge.isFalling()) {
                dropSpeed = DropSpeed.NORMAL;
                dropTimer += (getDropTime() - FAST_DROP_TIME);
            }
            if (leftEdge.isFalling())
                leftRepeat.setDown(false);
            if (rightEdge.isFalling())
                rightRepeat.setDown(false);
        }

        g.pushMatrix();
        g.translate(matrixOffsetX * pixelsPerTile, 0);

        if (state == GameState.PLAYING)
            step();

        g.background(32);
        matrix.draw(g);
        ctx.draw(g, false);

        TetrominoContext ghost = getHardDropPos();
        ghost.draw(g, true);

        // Matrix border
        g.noFill();
        g.stroke(128);
        g.rect(0, 0, Matrix.matrixWidth * pixelsPerTile, Matrix.matrixHeight * pixelsPerTile);

        preview.draw(g, false);
        if (hold != null)
            hold.draw(g, false);

        g.popMatrix();

        g.fill(255);
        g.text("Held", 20, 15);
        g.text("Up Next", 340, 15);

        g.text("Lines cleared:", 340, 120);
        g.text(String.valueOf(linesCleared), 340, 140);

        g.text("Level:", 340, 180);
        g.text(String.valueOf(getLevel() + 1), 340, 200);

        if (state == GameState.PAUSED || state == GameState.GAME_OVER) {
            g.noStroke();
            g.fill(0, 128);
            g.rect(0, 0, g.width, g.height);

            g.fill(255);

            String text = state == GameState.PAUSED ? "Paused" : "Game Over";

            g.text(text, g.width/2 - g.textWidth(text)/2, g.height/2 + g.textAscent()/2);
        }
    }
}
